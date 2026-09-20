package frc.lib.subsystems.motorInputs;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorInputsMeters extends MotorInputsAutoLogged {
    public double velocityMetersPerSecond = 0.0;
    public double positionMeters = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
    public double rotorPositionRotations = 0.0;
    public double[] velocities = new double[] {};
    public double[] timestamps = new double[] {};
    public double closedLoopError = 0.0;

    @Override
    public void setVelocity(double input) {
        velocityMetersPerSecond = input;
    }

    @Override
    public void setPosition(double input) {
        positionMeters = input;
    }

    @Override
    public void setVolts(double input) {
        appliedVolts = input;
    }

    @Override
    public void setCurrentStator(double input) {
        currentStatorAmps = input;
    }

    @Override
    public void setCurrentSupply(double input) {
        currentSupplyAmps = input;
    }

    @Override
    public void setRotorPosition(double input) {
        rotorPositionRotations = input;
    }

    @Override
    public void setVelocities(double[] input) {
        velocities = input;
    }

    @Override
    public void setTimestamps(double[] input) {
        timestamps = input;
    }

    @Override
    public double getVelocity() {
        return velocityMetersPerSecond;
    }

    @Override
    public double getPosition() {
        return positionMeters;
    }

    @Override
    public double getVolts() {
        return appliedVolts;
    }

    @Override
    public double getCurrentStator() {
        return currentStatorAmps;
    }

    @Override
    public double getCurrentSupply() {
        return currentSupplyAmps;
    }

    @Override
    public double getRotorPosition() {
        return rotorPositionRotations;
    }

    @Override
    public double[] getVelocities() {
        return velocities;
    }

    @Override
    public double[] getTimestamps() {
        return timestamps;
    }

    @Override
    public void setClosedLoopError(double input) {
        closedLoopError = input;
    }

    @Override
    public double getClosedLoopError() {
        return closedLoopError;
    }
}
