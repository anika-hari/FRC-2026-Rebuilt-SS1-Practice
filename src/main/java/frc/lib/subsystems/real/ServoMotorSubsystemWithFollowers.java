package frc.lib.subsystems.real;

import frc.lib.subsystems.CTREStatusSignalManager;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.motorInputs.MotorInputsAutoLogged;
import frc.lib.util.BatteryLogger;
import org.littletonrobotics.junction.Logger;

public class ServoMotorSubsystemWithFollowers<U extends MotorIO> extends ServoMotorSubsystem<U> {
    protected ServoMotorSubsystemWithFollowersConfig leaderConfig;
    protected ServoMotorSubsystemWithFollowersConfig.FollowerConfig[] followerConfigs;
    protected MotorInputsAutoLogged[] followerInputs;
    protected U[] followerIos;
    protected String[] loggingPrefixFollowers;

    public ServoMotorSubsystemWithFollowers(
            ServoMotorSubsystemWithFollowersConfig leaderConfig, U leaderIo, U[] followerIo) {
        super(leaderConfig, leaderIo);
        this.leaderConfig = leaderConfig;
        followerConfigs = leaderConfig.followers;

        assert followerConfigs.length == followerIo.length : "Length of follower configs/io not equal";
        followerIos = followerIo;
        followerInputs = new MotorInputsAutoLogged[followerConfigs.length];
        loggingPrefixFollowers = new String[followerConfigs.length];
        for (int i = 0; i < followerConfigs.length; i++) {
            followerInputs[i] = leaderConfig.unit.getInputs();
            MotorIO followerIO = followerIo[i];
            followerIO.follow(leaderConfig.talonCANID, followerConfigs[i].inverted);

            loggingPrefixFollowers[i] = getName() + "/follower" + i;
            CTREStatusSignalManager.register(followerIo[i], followerInputs[i]);
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        for (int i = 0; i < followerConfigs.length; i++) {
            // follower inputs are populated by CTREStatusSignalManager
            Logger.processInputs(loggingPrefixFollowers[i], followerInputs[i]);
        }
    }

    @Override
    protected void reportPowerUsage() {
        BatteryLogger.reportCurrentUsage(getName() + "/Leader", inputs.getCurrentSupply());
        for (int i = 0; i < followerInputs.length; i++) {
            BatteryLogger.reportCurrentUsage(getName() + "/Follower" + i, followerInputs[i].getCurrentSupply());
        }
    }

    @Override
    protected void setCurrentPositionAsZero() {
        super.setCurrentPositionAsZero();
        for (var follower : followerIos) {
            follower.setCurrentPositionAsZero();
        }
    }

    @Override
    public void setCurrentPosition(double positionUnits) {
        super.setCurrentPosition(positionUnits);
        for (var follower : followerIos) {
            follower.setCurrentPosition(positionUnits);
        }
    }

    @Override
    public double getPosition() {
        double averagePosition = inputs.getPosition();
        for (var followerInput : followerInputs) {
            averagePosition += followerInput.getPosition();
        }
        return averagePosition / (followerConfigs.length + 1);
    }

    // @Override
    // public double getCurrentVelocity() {
    //     double averagePosition = inputs.getVelocity();
    //     for (var followerInput : followerInputs) {
    //         averagePosition += followerInput.getVelocity();
    //     }
    //     Logger.recordOutput(getName() + "/currentVelocity", averagePosition / (followerConfigs.length + 1));
    //     return averagePosition / (followerConfigs.length + 1);
    // }
}
