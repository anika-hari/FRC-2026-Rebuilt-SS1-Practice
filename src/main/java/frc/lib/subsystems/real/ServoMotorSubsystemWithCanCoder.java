package frc.lib.subsystems.real;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystems.CTREStatusSignalManager;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.canDevice.CanCoderIO;
import frc.lib.subsystems.canDevice.CanCoderInputsAutoLogged;
import frc.lib.util.Util;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ServoMotorSubsystemWithCanCoder<
                U extends MotorIO, V extends CanCoderInputsAutoLogged, W extends CanCoderIO>
        extends ServoMotorSubsystem<U> {
    protected ServoMotorSubsystemWithCanCoderConfig conf;
    protected V cancoderInputs;
    protected W cancoderIO;
    protected boolean hasSetOffset = false;
    protected String loggingPrefixCancoder;

    public ServoMotorSubsystemWithCanCoder(
            ServoMotorSubsystemWithCanCoderConfig config, U io, V cancoderInputs, W cancoder) {
        super(config, io);
        this.conf = config;
        this.cancoderInputs = cancoderInputs;
        this.cancoderIO = cancoder;
        loggingPrefixCancoder = getName() + "/cancoder";
        CTREStatusSignalManager.register(this.cancoderIO, this.cancoderInputs);
    }

    @Override
    public void periodic() {
        super.periodic();

        // Cancoder inputs are refreshed centrally by CTREStatusSignalManager
        Logger.processInputs(loggingPrefixCancoder, cancoderInputs);

        // if (!this.conf.isFusedCancoder
        //         && !this.hasSetOffset
        //         && !Double.isNaN(cancoderInputs.absolutePositionRotations)) {
        //     io.setCurrentPosition(cancoderInputs.absolutePositionRotations * conf.cancoderToUnitsRatio);
        //     this.hasSetOffset = true;
        // }
    }

    @Override
    public void setCurrentPosition(double unitSetpoint) {
        cancoderIO.setPosition(unitSetpoint / conf.cancoderToUnitsRatio);
    }

    @Override
    public void setCurrentPositionAsZero() {
        cancoderIO.setPosition(0.0);
    }

    public void resetOffset() {
        // Don't set boolean above to left main thread still do it as well.
        io.setCurrentPosition(cancoderInputs.absolutePositionRotations * conf.cancoderToUnitsRatio);
    }

    public void resetOffset(int canCoderRotations) {
        io.setCurrentPosition(
                (cancoderInputs.absolutePositionRotations - canCoderRotations) * conf.cancoderToUnitsRatio);
    }

    public void zeroMagnetOffsetImpl() {
        cancoderIO.setMagnetOffset(0);
    }

    public Command zeroMagnetOffset() {
        return Commands.runOnce(this::zeroMagnetOffsetImpl);
    }

    private void printAbsoluteRotationsImpl() {
        cancoderIO.printAbsoluteRotations();
    }

    public Command printAbsoluteRotations() {
        return Commands.runOnce(this::printAbsoluteRotationsImpl);
    }

    public void setMultiTurnPosition(double maximumValuePossible) {
        double offset = (maximumValuePossible / conf.cancoderToUnitsRatio) - cancoderIO.getAbsolutePositionSlow();
        cancoderIO.setPosition(offset);
    }

    public void zeroMagnetOffsetImpl(double expectedZeroingPositionUnits) {
        cancoderIO.setMagnetOffset(0);
        double absoluteRotationsAtZeroingPosition = expectedZeroingPositionUnits * conf.cancoderToUnitsRatio;
        double magnetOffset = absoluteRotationsAtZeroingPosition - cancoderIO.getAbsolutePositionSlow();
        magnetOffset = Util.rangeModulo(
                magnetOffset,
                conf.canCoderConfig.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint,
                conf.canCoderConfig.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint - 1);

        cancoderIO.setMagnetOffset(magnetOffset);
        cancoderIO.setPosition(cancoderIO.getAbsolutePositionSlow() + Math.round(absoluteRotationsAtZeroingPosition));
    }

    public Command zeroMagnetOffset(DoubleSupplier expectedZeroingPositionUnits) {
        return run(() -> zeroMagnetOffsetImpl(expectedZeroingPositionUnits.getAsDouble()));
    }
}
