#!/usr/bin/env python3
"""
RIOLog Console Capture for FRC Robot / Simulation.

Captures console output to .riolog/console.log for Claude Code to analyze.

Usage:
    python scripts/riolog.py sim              # Launch simulation and capture output
    python scripts/riolog.py robot [address]  # Connect to robot via NetConsole protocol
"""

import argparse
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LOG_DIR = PROJECT_ROOT / ".riolog"
LOG_FILE = LOG_DIR / "console.log"

# NetConsole protocol constants
NETCONSOLE_PORT = 1741
TAG_ERROR = 11
TAG_INFO = 12
HEADER_FORMAT = ">Hb"  # big-endian: 2-byte length, 1-byte tag
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
INFO_HEADER_FORMAT = ">fH"  # float timestamp, ushort sequence
INFO_HEADER_SIZE = struct.calcsize(INFO_HEADER_FORMAT)
ERROR_HEADER_FORMAT = ">fHHiB"  # float ts, ushort seq, ushort numOcc, int errorCode, byte flags
ERROR_HEADER_SIZE = struct.calcsize(ERROR_HEADER_FORMAT)

KEEPALIVE_INTERVAL = 2.0
CONNECT_TIMEOUT = 3.0
RECONNECT_DELAY = 1.0

DEFAULT_ROBOT_ADDRESSES = [
    "roborio-3476-frc.local",
    "10.34.76.2",
]

shutdown_event = threading.Event()


def ensure_log_dir():
    LOG_DIR.mkdir(exist_ok=True)


def log_line(f, level, message):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    line = f"[{timestamp}] [{level}] {message}"
    print(line, flush=True)
    f.write(line + "\n")
    f.flush()


# ── Simulation Mode ──────────────────────────────────────────────────────────


def run_simulation(log_file):
    """Launch ./gradlew simulateJava and capture its output."""
    gradlew = PROJECT_ROOT / ("gradlew.bat" if sys.platform == "win32" else "gradlew")
    if not gradlew.exists():
        print(f"Error: {gradlew} not found", file=sys.stderr)
        sys.exit(1)

    log_line(log_file, "SYS", "Starting simulation: ./gradlew simulateJava")

    proc = subprocess.Popen(
        [str(gradlew), "simulateJava"],
        cwd=str(PROJECT_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
        text=True,
    )

    def on_shutdown(signum, frame):
        shutdown_event.set()
        proc.terminate()

    signal.signal(signal.SIGINT, on_shutdown)
    signal.signal(signal.SIGTERM, on_shutdown)

    try:
        for line in proc.stdout:
            line = line.rstrip("\n\r")
            if not line:
                continue
            # Heuristic: classify output as ERROR if it looks like one
            level = "ERROR" if _looks_like_error(line) else "INFO"
            log_line(log_file, level, line)
    except Exception as e:
        log_line(log_file, "SYS", f"Read error: {e}")
    finally:
        proc.wait()
        log_line(log_file, "SYS", f"Simulation exited with code {proc.returncode}")


def _looks_like_error(line):
    error_indicators = [
        "Exception", "Error", "FATAL", "at ", "Caused by:",
        "java.lang.", "WARNING", "WARN", "Traceback",
    ]
    return any(indicator in line for indicator in error_indicators)


# ── Robot NetConsole Mode ────────────────────────────────────────────────────


def _read_exact(sock_file, n):
    """Read exactly n bytes from socket file."""
    data = sock_file.read(n)
    if len(data) < n:
        raise ConnectionError("Connection closed")
    return data


def _read_string(sock_file):
    """Read a length-prefixed string (2-byte length + UTF-8 data)."""
    length_data = _read_exact(sock_file, 2)
    length = struct.unpack(">H", length_data)[0]
    if length == 0:
        return ""
    return _read_exact(sock_file, length).decode("utf-8", errors="replace")


def _parse_info(payload):
    """Parse TAG_INFO payload: timestamp + sequence + text."""
    if len(payload) < INFO_HEADER_SIZE:
        return None
    ts, seq = struct.unpack_from(INFO_HEADER_FORMAT, payload)
    text = payload[INFO_HEADER_SIZE:].decode("utf-8", errors="replace")
    return ts, text.rstrip("\n\r")


def _parse_error(payload):
    """Parse TAG_ERROR payload: header + 3 length-prefixed strings."""
    if len(payload) < ERROR_HEADER_SIZE:
        return None
    ts, seq, num_occ, error_code, flags = struct.unpack_from(ERROR_HEADER_FORMAT, payload)
    # Parse the three strings from the remaining payload
    import io
    buf = io.BytesIO(payload[ERROR_HEADER_SIZE:])
    sock_file = buf

    def read_str():
        ld = sock_file.read(2)
        if len(ld) < 2:
            return ""
        length = struct.unpack(">H", ld)[0]
        if length == 0:
            return ""
        return sock_file.read(length).decode("utf-8", errors="replace")

    details = read_str()
    location = read_str()
    call_stack = read_str()
    return ts, error_code, num_occ, flags, details, location, call_stack


def keepalive_thread(sock, lock):
    """Send keepalive null packets every 2 seconds."""
    while not shutdown_event.is_set():
        shutdown_event.wait(KEEPALIVE_INTERVAL)
        if shutdown_event.is_set():
            break
        try:
            with lock:
                sock.sendall(b"\x00\x00")
        except OSError:
            break


def connect_robot(address, log_file):
    """Connect to robot and read NetConsole messages."""
    signal.signal(signal.SIGINT, lambda s, f: shutdown_event.set())
    signal.signal(signal.SIGTERM, lambda s, f: shutdown_event.set())

    while not shutdown_event.is_set():
        sock = None
        try:
            log_line(log_file, "SYS", f"Connecting to {address}:{NETCONSOLE_PORT}...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(CONNECT_TIMEOUT)
            sock.connect((address, NETCONSOLE_PORT))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(None)

            log_line(log_file, "SYS", f"Connected to {address}")

            sock_file = sock.makefile("rb")
            lock = threading.Lock()

            ka_thread = threading.Thread(target=keepalive_thread, args=(sock, lock), daemon=True)
            ka_thread.start()

            while not shutdown_event.is_set():
                header_data = _read_exact(sock_file, HEADER_SIZE)
                length, tag = struct.unpack(HEADER_FORMAT, header_data)

                if length == 0:
                    continue  # keepalive response

                payload = _read_exact(sock_file, length)

                if tag == TAG_INFO:
                    result = _parse_info(payload)
                    if result:
                        ts, text = result
                        if text:
                            log_line(log_file, "INFO", text)

                elif tag == TAG_ERROR:
                    result = _parse_error(payload)
                    if result:
                        ts, code, num_occ, flags, details, location, call_stack = result
                        msg = f"[ErrorCode {code}] {details}"
                        if location:
                            msg += f"\n  at {location}"
                        if call_stack:
                            msg += f"\n{call_stack}"
                        log_line(log_file, "ERROR", msg)

        except ConnectionRefusedError:
            log_line(log_file, "SYS", f"Connection refused by {address}")
        except (ConnectionError, OSError) as e:
            log_line(log_file, "SYS", f"Connection lost: {e}")
        finally:
            if sock:
                try:
                    sock.close()
                except OSError:
                    pass

        if not shutdown_event.is_set():
            log_line(log_file, "SYS", f"Reconnecting in {RECONNECT_DELAY}s...")
            shutdown_event.wait(RECONNECT_DELAY)


def resolve_robot_address(address_arg):
    """Try to resolve robot address, falling back through defaults."""
    if address_arg:
        return address_arg

    for addr in DEFAULT_ROBOT_ADDRESSES:
        try:
            socket.getaddrinfo(addr, NETCONSOLE_PORT, socket.AF_INET, socket.SOCK_STREAM)
            return addr
        except socket.gaierror:
            continue

    # Return the IP fallback even if resolution failed (might work on the network)
    return DEFAULT_ROBOT_ADDRESSES[-1]


# ── Main ─────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(description="FRC RIOLog Console Capture")
    subparsers = parser.add_subparsers(dest="mode", required=True)

    subparsers.add_parser("sim", help="Launch simulation and capture output")

    robot_parser = subparsers.add_parser("robot", help="Connect to robot via NetConsole")
    robot_parser.add_argument("address", nargs="?", default=None,
                              help="Robot address (default: roborio-3476-frc.local or 10.34.76.2)")

    args = parser.parse_args()

    ensure_log_dir()

    with open(LOG_FILE, "w", encoding="utf-8") as f:
        log_line(f, "SYS", f"RIOLog capture started (mode={args.mode})")

        if args.mode == "sim":
            run_simulation(f)
        elif args.mode == "robot":
            address = resolve_robot_address(args.address)
            connect_robot(address, f)

        log_line(f, "SYS", "RIOLog capture stopped")


if __name__ == "__main__":
    main()
