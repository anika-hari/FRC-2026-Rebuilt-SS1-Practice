package frc.lib.subsystems;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Motor types compatible with TalonFX motor controllers for simulation.
 * Each enum value maps to a specific DCMotor factory method from WPILib.
 */
public enum TalonFXMotorType {
    KRAKEN_X60,
    KRAKEN_X60_FOC,
    KRAKEN_X44,
    KRAKEN_X44_FOC,
    FALCON_500,
    FALCON_500_FOC,
    MINION;

    /**
     * Gets the DCMotor model for this motor type.
     *
     * @param count The number of motors (typically 1 for most subsystems)
     * @return The DCMotor instance for simulation
     */
    public DCMotor getDCMotor(int count) {
        return switch (this) {
            case KRAKEN_X60 -> DCMotor.getKrakenX60(count);
            case KRAKEN_X60_FOC -> DCMotor.getKrakenX60Foc(count);
            case KRAKEN_X44 -> DCMotor.getKrakenX44(count);
            case KRAKEN_X44_FOC -> DCMotor.getKrakenX44Foc(count);
            case FALCON_500 -> DCMotor.getFalcon500(count);
            case FALCON_500_FOC -> DCMotor.getFalcon500Foc(count);
            case MINION -> DCMotor.getMinion(count);
        };
    }

    /**
     * Gets the TalonFXSimState.MotorType for this motor type.
     * Used to configure the CTRE simulation to use the correct motor model.
     *
     * @return The MotorType for TalonFXSimState
     */
    public TalonFXSimState.MotorType getSimMotorType() {
        return switch (this) {
            case KRAKEN_X60, KRAKEN_X60_FOC -> TalonFXSimState.MotorType.KrakenX60;
            case KRAKEN_X44, KRAKEN_X44_FOC, MINION -> TalonFXSimState.MotorType.KrakenX44;
            // Falcon 500 and Minion use KrakenX60 as a reasonable approximation
            case FALCON_500, FALCON_500_FOC -> TalonFXSimState.MotorType.KrakenX60;
        };
    }
}
