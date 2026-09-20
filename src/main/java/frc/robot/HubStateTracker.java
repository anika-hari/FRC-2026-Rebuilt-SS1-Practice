package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class HubStateTracker {
    public static enum Shift {
        AUTO,
        TRANSITION,
        SHIFT1,
        SHIFT2,
        SHIFT3,
        SHIFT4,
        ENDGAME
    }

    public static enum AutoResult {
        NONE,
        WIN,
        LOSE
    }

    private static String gameData = "";
    public static boolean doneWithAuto = false;
    public static boolean receivedData = false;
    public static Shift currentShift = Shift.AUTO;
    public static AutoResult wonAuto = AutoResult.NONE;
    private static Double untilNextActive;
    private static Double untilNextInactive;
    private static double untilNextShift = 20;
    private static double untilNextShiftFormatted = 0;
    private static String teamCode = "";
    public static Timer gameTimer = new Timer();

    public static void updateGameData() {
        if (wonAuto == AutoResult.NONE) {
            gameData = DriverStation.getGameSpecificMessage();
            if (gameData.length() > 0 && DriverStation.getAlliance().isPresent()) {
                receivedData = true;
                teamCode = DriverStation.getAlliance().get() == Alliance.Blue ? "B" : "R";
                if (gameData.equals(teamCode)) {
                    // we are going second, we won
                    wonAuto = AutoResult.WIN;
                } else {
                    // we are going first, we lost
                    wonAuto = AutoResult.LOSE;
                }
            }
        }

        // teleop length of this game is 140s
        double matchTime = 140 - gameTimer.get();
        if (!doneWithAuto) {
            // auto length of this game is 20s
            matchTime = 20 - gameTimer.get();
        }

        untilNextShift = getShiftRemainingDuration(currentShift, matchTime);

        if (untilNextShift <= 0 && doneWithAuto && currentShift != Shift.ENDGAME) {
            currentShift = Shift.values()[currentShift.ordinal() + 1]; // increment shift
            untilNextShift = getShiftRemainingDuration(currentShift, matchTime);
        }

        if (isActiveShift(currentShift)) {
            untilNextActive = 0.0;
            untilNextInactive = untilNextShift;
        } else {
            untilNextActive = untilNextShift;
            untilNextInactive = 0.0;
        }

        // "double shift" special cases

        if (wonAuto == AutoResult.LOSE && currentShift == Shift.TRANSITION) {
            untilNextInactive += 25;
        }

        if (wonAuto == AutoResult.WIN && currentShift == Shift.SHIFT4) {
            untilNextInactive += 30;
        }

        Logger.recordOutput("HubStateTracker/currentShift", currentShift);
        Logger.recordOutput("HubStateTracker/untilNextShiftS", untilNextShift);
        Logger.recordOutput("HubStateTracker/untilNextActiveS", untilNextActive);
        Logger.recordOutput("HubStateTracker/untilNextInactiveS", untilNextInactive);
        Logger.recordOutput(
                "HubStateTracker/untilNextShiftFormatted",
                Math.max(Math.ceil(untilNextInactive * 10) / 10, Math.ceil(untilNextActive * 10) / 10));
        Logger.recordOutput("HubStateTracker/matchTimeS", matchTime);
        Logger.recordOutput("HubStateTracker/isActiveShift", isActiveShift(currentShift));
        Logger.recordOutput("HubStateTracker/teamCode", teamCode);
        Logger.recordOutput("HubStateTracker/gameData", gameData);
        Logger.recordOutput("HubStateTracker/wonAuto", wonAuto);

        untilNextShiftFormatted =
                Math.max(Math.ceil(untilNextInactive * 10) / 10, Math.ceil(untilNextActive * 10) / 10);

        if (untilNextActive < 3 && untilNextActive > 0) {
            if (untilNextActive % 0.5 > 0.20) {
                SmartDashboard.putString("Robot/shootAlarm", "!!SHOOT NOW!!");
            } else {
                SmartDashboard.putString("Robot/shootAlarm", "");
            }
        }
    }

    public static boolean isActiveShift(Shift shift) {
        if (wonAuto == AutoResult.WIN) {
            switch (shift) {
                case AUTO:
                case TRANSITION:
                case SHIFT2:
                case SHIFT4:
                case ENDGAME:
                    return true;
                default:
                    return false;
            }
        } else {
            switch (shift) {
                case AUTO:
                case TRANSITION:
                case SHIFT1:
                case SHIFT3:
                case ENDGAME:
                    return true;
                default:
                    return false;
            }
        }
    }

    public static AutoResult whoWonAuto() {
        return wonAuto;
    }

    public static void setAutoWinner(AutoResult result) {
        wonAuto = result;
    }

    public static Optional<Double> getUntilNextActive() {
        return Optional.ofNullable(untilNextActive);
    }

    public static Optional<Double> getUntilNextInactive() {
        return Optional.ofNullable(untilNextInactive);
    }

    public static double getUntilNextShift() {
        return untilNextShift;
    }

    public static double getUntilNextShiftFormatted() {
        return untilNextShiftFormatted;
    }

    public static boolean cantShoot() {
        return !isActiveShift(currentShift)
                && getUntilNextActive().orElse(25.0) > 2
                && getUntilNextActive().orElse(25.0) < 24;
    }

    private static double getShiftRemainingDuration(Shift currentShift, double matchTime) {
        switch (currentShift) {
            case AUTO:
                if (doneWithAuto) {
                    return 0;
                }
                return matchTime;
            case TRANSITION:
                return matchTime - 130; // 2:10
            case SHIFT1:
                return matchTime - 105; // 1:45
            case SHIFT2:
                return matchTime - 80; // 1:20
            case SHIFT3:
                return matchTime - 55; // 0:55
            case SHIFT4:
                return matchTime - 30; // 0:30
            case ENDGAME:
                return matchTime;
            default:
                return matchTime;
        }
    }
}
