package org.team1540.robot2024.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMPS = 0.0;
        public double[] voltage = new double[]{};
        public double[] currentAmps = new double[]{};
        public double[] tempCelsius = new double[]{};
        public boolean atUpperLimit = false;
        public boolean atLowerLimit = false;
        public double flipperAngleDegrees = 0.0;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setSetpointMeters(double position) {}

    default void setVoltage(double voltage) {}

    default void setBrakeMode(boolean isBrakeMode) {}

    default void setFlipper(boolean flipped) {}

    default void configPID(double kP, double kI, double kD) {}
}


