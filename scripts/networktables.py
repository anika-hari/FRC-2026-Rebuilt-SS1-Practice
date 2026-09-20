#!/usr/bin/env python3
"""
NetworkTables 4 Capture for FRC Robot / Simulation.

Connects to NT4 via WebSocket and captures topic data to .ntcapture/ for Claude Code to analyze.

Usage:
    python scripts/networktables.py sim                          # Snapshot from simulation
    python scripts/networktables.py sim --stream                 # Continuous capture from sim
    python scripts/networktables.py sim --filter Drive            # Only Drive subsystem
    python scripts/networktables.py robot                        # Snapshot from real robot
    python scripts/networktables.py robot 10.34.76.2 --stream    # Stream from robot at IP
"""

import argparse
import asyncio
import io
import json
import signal
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

# Check dependencies before importing
def check_dependencies():
    missing = []
    try:
        import websockets  # noqa: F401
    except ImportError:
        missing.append("websockets")
    try:
        import msgpack  # noqa: F401
    except ImportError:
        missing.append("msgpack")
    if missing:
        print(f"Missing required packages: {', '.join(missing)}")
        print(f"Install with: pip install {' '.join(missing)}")
        sys.exit(1)

check_dependencies()

import msgpack
import websockets


def verify_msgpack():
    """Verify msgpack can round-trip NT4-style records on this Python version."""
    test_record = [42, 1000000, 1, 3.14]
    packed = msgpack.packb(test_record)
    unpacker = msgpack.Unpacker(io.BytesIO(packed), raw=False, strict_map_key=False)
    result = list(unpacker)
    assert len(result) == 1 and result[0] == test_record, f"msgpack self-test failed: {result}"


verify_msgpack()

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CAPTURE_DIR = PROJECT_ROOT / ".ntcapture"
TOPICS_FILE = CAPTURE_DIR / "topics.log"
SNAPSHOT_FILE = CAPTURE_DIR / "snapshot.log"
CHANGES_FILE = CAPTURE_DIR / "changes.log"

NT4_PORT = 5810
NT4_SUBPROTOCOL = "v4.1.networktables.first.wpi.edu"
NT4_SUBPROTOCOL_FALLBACK = "networktables.first.wpi.edu"

DEFAULT_ROBOT_ADDRESSES = [
    "roborio-3476-frc.local",
    "10.34.76.2",
]

TYPE_NAMES = {
    0: "boolean", 1: "double", 2: "int", 3: "float", 4: "string", 5: "raw",
    16: "boolean[]", 17: "double[]", 18: "int[]", 19: "float[]", 20: "string[]",
}

shutdown_event = asyncio.Event()


@dataclass
class TopicInfo:
    name: str
    id: int
    type_str: str
    type_id: int
    properties: dict = field(default_factory=dict)


@dataclass
class Options:
    host: str = "localhost"
    port: int = NT4_PORT
    stream: bool = False
    duration: float = 0
    filters: list = field(default_factory=list)
    excludes: list = field(default_factory=list)
    mode: str = "sim"


# ── Logging ──────────────────────────────────────────────────────────────────


def timestamp_str():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def log_line(f, level, message):
    line = f"[{timestamp_str()}] [{level}] {message}"
    print(line, flush=True)
    if f:
        f.write(line + "\n")
        f.flush()


# ── Value formatting ─────────────────────────────────────────────────────────


def format_value(value, type_id=-1):
    """Format a value for human-readable display."""
    if isinstance(value, bytes):
        return f"<binary {len(value)} bytes>"
    if isinstance(value, float):
        return f"{value:.6f}"
    if isinstance(value, (list, tuple)):
        if len(value) > 20:
            items = [format_value(v) for v in value[:20]]
            return f"[{', '.join(items)}, ... ({len(value)} total)]"
        return f"[{', '.join(format_value(v) for v in value)}]"
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


# ── Topic filtering ──────────────────────────────────────────────────────────


def display_name(topic_name):
    """Strip /AdvantageKit/ prefix for display, and leading slash."""
    if topic_name.startswith("/AdvantageKit/"):
        return topic_name[len("/AdvantageKit/"):]
    return topic_name.lstrip("/")


def subsystem_of(name):
    """Extract the subsystem (first path segment) from a display name."""
    parts = name.split("/", 1)
    return parts[0] if parts else name


def should_include(topic_name, options):
    """Check if topic passes filter/exclude rules."""
    dn = display_name(topic_name)

    for exc in options.excludes:
        if dn.startswith(exc) or topic_name.lstrip("/").startswith(exc):
            return False

    if not options.filters:
        return True

    for filt in options.filters:
        if dn.startswith(filt) or topic_name.lstrip("/").startswith(filt):
            return True
    return False


# ── Snapshot writing ─────────────────────────────────────────────────────────


def write_topics_log(topic_registry):
    """Write topics.log with all announced topics."""
    with open(TOPICS_FILE, "w", encoding="utf-8") as f:
        f.write(f"[{timestamp_str()}] [SYS] Topic list ({len(topic_registry)} topics)\n")
        for tid in sorted(topic_registry, key=lambda t: topic_registry[t].name):
            info = topic_registry[tid]
            f.write(f"[{timestamp_str()}] [TOPIC] id={info.id} name={info.name} type={info.type_str}\n")
        f.write(f"[{timestamp_str()}] [SYS] End topic list\n")


def write_snapshot(topic_registry, topic_values, options):
    """Write snapshot.log grouped by subsystem."""
    # Build display data: {subsystem: [(short_name, formatted_value)]}
    subsystems = {}
    for tid, (ts_us, value) in sorted(topic_values.items(), key=lambda x: topic_registry.get(x[0], TopicInfo("", 0, "", -1)).name):
        info = topic_registry.get(tid)
        if not info:
            continue
        if not should_include(info.name, options):
            continue

        dn = display_name(info.name)
        sub = subsystem_of(dn)
        short = dn[len(sub):].lstrip("/") if "/" in dn else dn
        if not short:
            short = sub

        subsystems.setdefault(sub, []).append((short, format_value(value, info.type_id)))

    with open(SNAPSHOT_FILE, "w", encoding="utf-8") as f:
        total = sum(len(v) for v in subsystems.values())
        f.write(f"[{timestamp_str()}] [SYS] NetworkTables Snapshot ({total} topics, {options.mode} mode)\n\n")

        for sub in sorted(subsystems):
            f.write(f"=== {sub} ===\n")
            for short_name, val in subsystems[sub]:
                f.write(f"  {short_name:<40s} = {val}\n")
            f.write("\n")

        f.write(f"[{timestamp_str()}] [SYS] End snapshot\n")

    print(f"[{timestamp_str()}] [SYS] Snapshot written: {total} topics across {len(subsystems)} subsystems", flush=True)


# ── NT4 Client ───────────────────────────────────────────────────────────────


async def nt4_client(options):
    """Main NT4 WebSocket client loop with reconnection."""
    topic_registry = {}
    topic_values = {}
    prev_values = {}
    pending_values = {}  # Buffer for values received before their topic announce
    changes_file = None

    # Diagnostic counters
    text_msg_count = 0
    binary_frame_count = 0
    total_records = 0
    unknown_tid_count = 0
    parse_error_count = 0

    CAPTURE_DIR.mkdir(exist_ok=True)

    if options.stream:
        changes_file = open(CHANGES_FILE, "w", encoding="utf-8")
        log_line(changes_file, "SYS", f"NetworkTables stream started ({options.mode} mode)")

    uri = f"ws://{options.host}:{options.port}/nt/claude-code"
    start_time = time.monotonic()

    while not shutdown_event.is_set():
        try:
            log_line(changes_file, "SYS", f"Connecting to {uri}...")

            async with websockets.connect(
                uri,
                subprotocols=[NT4_SUBPROTOCOL, NT4_SUBPROTOCOL_FALLBACK],
                compression=None,
                ping_interval=None,
                ping_timeout=None,
                max_size=2 ** 20,
                open_timeout=5,
                close_timeout=2,
            ) as ws:
                log_line(changes_file, "SYS", f"Connected (protocol: {ws.subprotocol})")
                extensions = getattr(ws, 'extensions', None) or getattr(getattr(ws, 'protocol', None), 'extensions', [])
                log_line(changes_file, "SYS", f"Extensions: {extensions}")

                # Subscribe to all topics with prefix match
                subscribe_msg = [{
                    "method": "subscribe",
                    "params": {
                        "topics": [""],
                        "subuid": 1,
                        "options": {
                            "periodic": 0.1,
                            "all": False,
                            "topicsonly": False,
                            "prefix": True,
                        },
                    },
                }]
                await ws.send(json.dumps(subscribe_msg))

                last_announce_time = time.monotonic()
                last_value_time = 0.0
                last_snapshot_time = 0.0
                initial_done = False

                while not shutdown_event.is_set():
                    # Check duration limit
                    if options.duration > 0 and (time.monotonic() - start_time) >= options.duration:
                        log_line(changes_file, "SYS", f"Duration limit ({options.duration}s) reached")
                        shutdown_event.set()
                        break

                    try:
                        message = await asyncio.wait_for(ws.recv(), timeout=0.5)
                    except asyncio.TimeoutError:
                        # In snapshot mode, check if we've settled
                        if not options.stream and not initial_done:
                            now = time.monotonic()
                            elapsed_since_announce = now - last_announce_time
                            elapsed_since_value = now - last_value_time if last_value_time > 0 else float('inf')
                            have_values = len(topic_values) > 0

                            if topic_registry and elapsed_since_announce > 1.5:
                                if have_values and elapsed_since_value > 1.0:
                                    # Both announces and values have settled
                                    initial_done = True
                                    write_topics_log(topic_registry)
                                    write_snapshot(topic_registry, topic_values, options)
                                    shutdown_event.set()
                                    break
                                elif now - start_time > 10.0:
                                    # Hard timeout — write what we have with a warning
                                    log_line(changes_file, "WARN",
                                             f"Snapshot timeout: {len(topic_registry)} topics announced, "
                                             f"{len(topic_values)} values captured, "
                                             f"{binary_frame_count} binary frames, "
                                             f"{unknown_tid_count} unknown tids, "
                                             f"{parse_error_count} parse errors")
                                    initial_done = True
                                    write_topics_log(topic_registry)
                                    write_snapshot(topic_registry, topic_values, options)
                                    shutdown_event.set()
                                    break
                        continue

                    if isinstance(message, str):
                        # JSON text frame: announce/unannounce
                        text_msg_count += 1
                        try:
                            msgs = json.loads(message)
                            for msg in msgs:
                                method = msg.get("method")
                                params = msg.get("params", {})

                                if method == "announce":
                                    tid = params["id"]
                                    type_str = params.get("type", "unknown")
                                    type_id = next(
                                        (k for k, v in TYPE_NAMES.items() if v == type_str),
                                        -1,
                                    )
                                    topic_registry[tid] = TopicInfo(
                                        name=params["name"],
                                        id=tid,
                                        type_str=type_str,
                                        type_id=type_id,
                                        properties=params.get("properties", {}),
                                    )
                                    last_announce_time = time.monotonic()

                                    # Flush any pending values that arrived before this announce
                                    if tid in pending_values:
                                        pts_us, ptype_id, pvalue = pending_values.pop(tid)
                                        if should_include(params["name"], options):
                                            topic_values[tid] = (pts_us, pvalue)
                                            last_value_time = time.monotonic()

                                elif method == "unannounce":
                                    tid = params.get("id")
                                    if tid is not None:
                                        topic_registry.pop(tid, None)
                                        topic_values.pop(tid, None)
                                        prev_values.pop(tid, None)
                        except json.JSONDecodeError as e:
                            log_line(changes_file, "ERR", f"JSON parse error: {e}")

                    elif isinstance(message, bytes):
                        # Binary frame: msgpack value updates
                        binary_frame_count += 1
                        if binary_frame_count <= 5:
                            log_line(changes_file, "DBG",
                                     f"Binary frame #{binary_frame_count}: {len(message)} bytes, "
                                     f"hex={message[:64].hex()}")

                        try:
                            unpacker = msgpack.Unpacker(io.BytesIO(message), raw=False, strict_map_key=False)
                            records = list(unpacker)
                        except (msgpack.UnpackException, ValueError) as e:
                            parse_error_count += 1
                            log_line(changes_file, "ERR",
                                     f"msgpack parse error: {e} "
                                     f"(frame {len(message)} bytes, hex={message[:32].hex()})")
                            continue

                        for record in records:
                            total_records += 1
                            if not isinstance(record, (list, tuple)) or len(record) < 4:
                                if total_records <= 5:
                                    log_line(changes_file, "DBG",
                                             f"Skipped record (type={type(record).__name__}, "
                                             f"len={len(record) if isinstance(record, (list, tuple)) else 'N/A'}): "
                                             f"{repr(record)[:100]}")
                                continue

                            tid, ts_us, type_id, value = record[0], record[1], record[2], record[3]

                            if tid == -1:
                                continue

                            if tid not in topic_registry:
                                unknown_tid_count += 1
                                # Buffer for later — announce may arrive after value
                                if len(pending_values) < 10000:
                                    pending_values[tid] = (ts_us, type_id, value)
                                if unknown_tid_count <= 10:
                                    log_line(changes_file, "DBG",
                                             f"Unknown tid={tid} (registry has {len(topic_registry)} topics), buffered")
                                continue

                            info = topic_registry[tid]
                            if not should_include(info.name, options):
                                continue

                            topic_values[tid] = (ts_us, value)
                            last_value_time = time.monotonic()

                            # Log changes in stream mode
                            if options.stream and changes_file:
                                old_val = prev_values.get(tid)
                                if old_val != value:
                                    dn = display_name(info.name)
                                    log_line(changes_file, "CHG", f"{dn} = {format_value(value, info.type_id)}")
                                    prev_values[tid] = value

                    else:
                        log_line(changes_file, "WARN",
                                 f"Unknown message type: {type(message).__name__}")

                    # Periodic snapshot in stream mode
                    if options.stream and topic_registry:
                        now = time.monotonic()
                        if now - last_snapshot_time >= 5.0:
                            write_topics_log(topic_registry)
                            write_snapshot(topic_registry, topic_values, options)
                            last_snapshot_time = now

        except (ConnectionRefusedError, OSError) as e:
            log_line(changes_file, "SYS", f"Connection failed: {e}")
        except websockets.exceptions.ConnectionClosed as e:
            log_line(changes_file, "SYS", f"Connection closed: {e}")
        except Exception as e:
            log_line(changes_file, "SYS", f"Error: {e}")

        if shutdown_event.is_set():
            break

        log_line(changes_file, "SYS", "Reconnecting in 2s...")
        try:
            await asyncio.wait_for(shutdown_event.wait(), timeout=2.0)
        except asyncio.TimeoutError:
            pass

    # Session statistics
    log_line(changes_file, "SYS",
             f"Session stats: {text_msg_count} text frames, {binary_frame_count} binary frames, "
             f"{total_records} records, {unknown_tid_count} unknown tids, "
             f"{parse_error_count} parse errors, {len(topic_registry)} topics, "
             f"{len(topic_values)} values captured, {len(pending_values)} still pending")

    # Final snapshot on shutdown
    if topic_registry and topic_values:
        write_topics_log(topic_registry)
        write_snapshot(topic_registry, topic_values, options)

    if changes_file:
        log_line(changes_file, "SYS", "NetworkTables stream stopped")
        changes_file.close()


# ── Main ─────────────────────────────────────────────────────────────────────


def resolve_robot_address(address_arg):
    import socket
    if address_arg:
        return address_arg
    for addr in DEFAULT_ROBOT_ADDRESSES:
        try:
            socket.getaddrinfo(addr, NT4_PORT, socket.AF_INET, socket.SOCK_STREAM)
            return addr
        except socket.gaierror:
            continue
    return DEFAULT_ROBOT_ADDRESSES[-1]


def main():
    parser = argparse.ArgumentParser(description="FRC NetworkTables Capture")
    subparsers = parser.add_subparsers(dest="mode", required=True)

    sim_parser = subparsers.add_parser("sim", help="Connect to simulation NT on localhost")
    sim_parser.add_argument("--stream", action="store_true", help="Continuous capture")
    sim_parser.add_argument("--duration", type=float, default=0, help="Stop after N seconds")
    sim_parser.add_argument("--filter", action="append", default=[], help="Include topics with prefix")
    sim_parser.add_argument("--exclude", action="append", default=[], help="Exclude topics with prefix")

    robot_parser = subparsers.add_parser("robot", help="Connect to real robot NT")
    robot_parser.add_argument("address", nargs="?", default=None, help="Robot address")
    robot_parser.add_argument("--stream", action="store_true", help="Continuous capture")
    robot_parser.add_argument("--duration", type=float, default=0, help="Stop after N seconds")
    robot_parser.add_argument("--filter", action="append", default=[], help="Include topics with prefix")
    robot_parser.add_argument("--exclude", action="append", default=[], help="Exclude topics with prefix")

    args = parser.parse_args()

    opts = Options(
        stream=args.stream,
        duration=args.duration,
        filters=args.filter,
        excludes=args.exclude,
        mode=args.mode,
    )

    if args.mode == "sim":
        opts.host = "localhost"
    elif args.mode == "robot":
        opts.host = resolve_robot_address(getattr(args, "address", None))

    opts.port = NT4_PORT

    # Handle Ctrl+C gracefully
    def handle_signal():
        shutdown_event.set()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # On Windows, SIGINT is handled via KeyboardInterrupt
    if sys.platform != "win32":
        loop.add_signal_handler(signal.SIGINT, handle_signal)
        loop.add_signal_handler(signal.SIGTERM, handle_signal)

    try:
        loop.run_until_complete(nt4_client(opts))
    except KeyboardInterrupt:
        shutdown_event.set()
        # Give a moment for cleanup
        loop.run_until_complete(asyncio.sleep(0.1))
    finally:
        loop.close()

    print(f"\nCapture files in: {CAPTURE_DIR}", flush=True)


if __name__ == "__main__":
    main()
