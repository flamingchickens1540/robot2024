package org.team1540.robot2024.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    class IndexerIOInputs {
        public double intakeVoltage;
        public double intakeCurrentAmps;
        public double intakeVelocityRPM;
        public double feederVoltage;
        public double feederCurrentAmps;
        public double feederVelocityRPM;
        public boolean noteInIntake = false;
        public double setpoint;
        public double feederVelocityRadPerSec;
        public double feederPositionError;
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(IndexerIOInputs inputs) {}

    default void setIntakeVoltage(double volts) {}

    default void setFeederVoltage(double volts) {}

    default void setFeederVelocity(double velocity) {}

    default void configurePID(double p, double i, double d) {}

}
