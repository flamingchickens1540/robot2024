package org.team1540.robot2024.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMPS = 0.0;
        public double voltage = 0.0;
        public double[] current = new double[]{};
        public boolean atUpperLimit = false;
        public boolean atLowerLimit = false;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setSetpointMeters(double position) {}

    default void setVoltage(double voltage) {}
}


