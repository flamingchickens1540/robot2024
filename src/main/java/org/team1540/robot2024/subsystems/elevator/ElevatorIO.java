package org.team1540.robot2024.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double positionRots = 0.0;
        public double velocityRPM = 0.0;
        public double voltage = 0.0;
        public double[] current = new double[] {};
        public boolean upperLimit = false;
        public boolean lowerLimit = false;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void setSetpoint(double position) {}

    default void stop() {}


}


