package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.time.RobotTime;
import frc.lib.util.COColor;
import frc.lib.util.ConcurrentTimeInterpolatableBuffer;
import frc.lib.util.LoggedTunableNumber;
import frc.robot.game_util.FieldConstants;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

public class RobotState {

    private static final AtomicReference<Pose2d> latestVisionPose = new AtomicReference<>(Pose2d.kZero);
    private static final AtomicReference<Double> latestVisionTimestamp =
            new AtomicReference<>(RobotTime.getTimestampSeconds());

    public static boolean isDoneShooting = false;

    public static void updateVisionPose(Pose2d pose, double timestamp) {
        latestVisionPose.set(pose);
        latestVisionTimestamp.set(timestamp);
    }

    public static Optional<Pose2d> getVisionPose(double thresholdSeconds) {
        if (RobotTime.getTimestampSeconds() - latestVisionTimestamp.get() < thresholdSeconds) {
            return Optional.of(latestVisionPose.get());
        }
        return Optional.empty();
    }

    public static Optional<Pose2d> getVisionPose() {
        return getVisionPose(Double.MAX_VALUE);
    }

    public static final double LOOKBACK_TIME = 1.0;
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot =
            ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);
    // Current robot-relative chassis speeds (measured from encoders)
    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    // Current field-relative chassis speeds (measured from encoders)
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    // Desired robot-relative chassis speeds (set by control systems)
    private final AtomicReference<ChassisSpeeds> desiredRobotRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    // Desired field-relative chassis speeds (set by control systems)
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> sotmSetpointFieldRelativeSpeeds =
            new AtomicReference<>(new ChassisSpeeds());
    private double goalTangentialSpeedMetersPerSecond = 0.0;
    private double lastUsedEstimateTimestamp = 0;
    private Pose2d lastUsedEstimatePose = Pose2d.kZero;
    private final ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> driveRollRads =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> accelX =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private final ConcurrentTimeInterpolatableBuffer<Double> accelY =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(LOOKBACK_TIME);
    private static final AtomicReference<Pose2d> globalPose = new AtomicReference<>(Pose2d.kZero);

    public static final LoggedTunableNumber autoMaxVelocity = new LoggedTunableNumber("Auto/maxVelo", 4); // m/s
    public static final LoggedTunableNumber autoMaxAccel = new LoggedTunableNumber("Auto/maxAccel", 200); // m/s^2
    public static final LoggedTunableNumber autoMaxRotationalVelocity =
            new LoggedTunableNumber("Auto/maxRotationalVelo", 10000.0000); // deg/s
    public static final LoggedTunableNumber autoMaxRotationalAccel =
            new LoggedTunableNumber("Auto/maxRotationalAccel", 10000.0000); // deg/s^2

    public static Pose2d getGlobalPose() {
        return globalPose.get();
    }

    public static void updateGlobalPose(Pose2d pose) {
        globalPose.set(pose);
    }

    public static boolean isInTrench() {
        return FieldConstants.Trench.trenchArmBlueLeft.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.Trench.trenchArmBlueRight.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.Trench.trenchArmRedLeft.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.Trench.trenchArmRedRight.contains(
                        getGlobalPose().getX(), getGlobalPose().getY());
    }

    public static boolean isInScoringZone() {
        return FieldConstants.ScoringZones.scoringZoneBlue.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.ScoringZones.scoringZoneRed.contains(
                        getGlobalPose().getX(), getGlobalPose().getY());
    }

    public static BooleanSupplier isInPreSpinUpZone() {
        return () -> FieldConstants.ScoringZones.spinUpZoneBlue.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.ScoringZones.spinUpZoneRed.contains(
                        getGlobalPose().getX(), getGlobalPose().getY());
    }

    public static boolean isInDepot() {
        return FieldConstants.Depot.depotBlue.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.Depot.depotRed.contains(
                        getGlobalPose().getX(), getGlobalPose().getY());
    }

    public static boolean isOnBumps() {
        return FieldConstants.Bumps.bumpBlueLeft.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.Bumps.bumpBlueRight.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.Bumps.bumpRedLeft.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.Bumps.bumpRedRight.contains(
                        getGlobalPose().getX(), getGlobalPose().getY());
    }

    public static boolean isNearBumps() {
        return FieldConstants.ExtendedBumps.extendedBumpBlueLeft.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.ExtendedBumps.extendedBumpBlueRight.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.ExtendedBumps.extendedBumpRedLeft.contains(
                        getGlobalPose().getX(), getGlobalPose().getY())
                || FieldConstants.ExtendedBumps.extendedBumpRedRight.contains(
                        getGlobalPose().getX(), getGlobalPose().getY());
    }

    public static boolean willCrossBumpInTheFuture(Pose2d futurePose) {
        // Use in conjuction with isNearBumps to not activate while going under trench
        double blueBump = FieldConstants.hubCenterToAllianceWall;
        double redBump = FieldConstants.fieldLengthMeters - FieldConstants.hubCenterToAllianceWall;
        if (getGlobalPose().getX() > blueBump && futurePose.getX() < blueBump) {

            return true;
        }
        if (getGlobalPose().getX() < blueBump && futurePose.getX() > blueBump) {

            return true;
        }
        if (getGlobalPose().getX() > redBump && futurePose.getX() < redBump) {

            return true;
        }

        if (getGlobalPose().getX() < redBump && futurePose.getX() > redBump) {
            return true;
        }
        return false; // temp
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void setRobotToHubTangentialSpeed(double speedMetersPerSecond) {
        this.goalTangentialSpeedMetersPerSecond = speedMetersPerSecond;
    }

    public double getRobotToHubTangentialSpeed() {
        return this.goalTangentialSpeedMetersPerSecond;
    }

    public void addDriveMotionMeasurements(
            double timestamp,
            double angularRollRadsPerS,
            double angularPitchRadsPerS,
            double angularYawRadsPerS,
            double pitchRads,
            double rollRads,
            double accelX,
            double accelY,
            ChassisSpeeds desiredRobotRelativeChassisSpeeds,
            ChassisSpeeds desiredFieldRelativeSpeeds,
            ChassisSpeeds measuredSpeeds,
            ChassisSpeeds measuredFieldRelativeSpeeds,
            ChassisSpeeds fusedFieldRelativeSpeeds) {
        this.driveRollAngularVelocity.addSample(timestamp, angularRollRadsPerS);
        this.drivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
        this.driveYawAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        this.drivePitchRads.addSample(timestamp, pitchRads);
        this.driveRollRads.addSample(timestamp, rollRads);
        this.accelY.addSample(timestamp, accelY);
        this.accelX.addSample(timestamp, accelX);
        this.desiredRobotRelativeChassisSpeeds.set(desiredRobotRelativeChassisSpeeds);
        this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
        this.measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
        this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
        this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getLatest();
    }

    /**
     * Predicts robot's future pose based on current velocity.
     *
     * @param lookaheadTimeS How far ahead to predict (seconds)
     * @return Predicted pose
     */
    public Pose2d getPredictedFieldToRobot(double lookaheadTimeS) {
        var maybeFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot = maybeFieldToRobot == null ? Pose2d.kZero : maybeFieldToRobot.getValue();
        var delta = getLatestRobotRelativeChassisSpeed();
        delta = delta.times(lookaheadTimeS);
        return fieldToRobot.exp(
                new Twist2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond));
    }

    /**
     * Like getPredictedFieldToRobot but caps negative velocities to zero. Used for non-holonomic path
     * planning.
     */
    public Pose2d getPredictedCappedFieldToRobot(double lookaheadTimeS) {
        var maybeFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot = maybeFieldToRobot == null ? Pose2d.kZero : maybeFieldToRobot.getValue();
        var delta = getLatestRobotRelativeChassisSpeed();
        delta = delta.times(lookaheadTimeS);
        return fieldToRobot.exp(new Twist2d(
                Math.max(0.0, delta.vxMetersPerSecond),
                Math.max(0.0, delta.vyMetersPerSecond),
                delta.omegaRadiansPerSecond));
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredRobotRelativeChassisSpeeds() {
        return desiredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public void setSotmSetpointVelocity(ChassisSpeeds speeds) {
        sotmSetpointFieldRelativeSpeeds.set(speeds);
    }

    public ChassisSpeeds getSotmSetpointVelocity() {
        return sotmSetpointFieldRelativeSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestRobotRelativeChassisSpeed();
        speeds.omegaRadiansPerSecond = getLatestFusedFieldRelativeChassisSpeed().omegaRadiansPerSecond;
        return speeds;
    }

    private static final AtomicReference<COColor> rightLedState = new AtomicReference<>(COColor.kCOOrangePure);

    public static void setRightLedState(COColor state) {
        rightLedState.set(state);
    }

    public static COColor getRightLedState() {
        return rightLedState.get();
    }

    private static final AtomicReference<COColor> leftLedState = new AtomicReference<>(COColor.kCOOrangePure);

    public static void setLeftLedState(COColor state) {
        leftLedState.set(state);
    }

    public static COColor getLeftLedState() {
        return leftLedState.get();
    }
}
