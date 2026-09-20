package frc.lib.subsystems;

import frc.lib.subsystems.motorInputs.MotorInputsAutoLogged;
import frc.lib.subsystems.motorInputs.MotorInputsGenericAutoLogged;
import frc.lib.subsystems.motorInputs.MotorInputsMetersAutoLogged;
import frc.lib.subsystems.motorInputs.MotorInputsRadiansAutoLogged;

public enum SubsystemUnit {
    METERS("Meters"),
    RADIANS("Radians"),
    GENERIC("Units");

    private final String unitName;

    SubsystemUnit(String unitName) {
        this.unitName = unitName;
    }

    public String getUnitName() {
        return unitName;
    }

    public MotorInputsAutoLogged getInputs() {
        switch (this) {
            case METERS:
                return new MotorInputsMetersAutoLogged();
            case RADIANS:
                return new MotorInputsRadiansAutoLogged();
            default:
                return new MotorInputsGenericAutoLogged();
        }
    }
}
