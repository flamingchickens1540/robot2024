package org.team1540.robot2024.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    class IndexerIOInputs {
        public double intakeVoltage = 0.0;
        public double intakeCurrentAmps = 0.01;
        public double intakeVelocityRPS = 0.0;
        public double intakeTempCelsius = 0.0;
        public double feederVoltage = 0.0;
        public double feederCurrentAmps = 0.0;
        public double feederVelocityRPS = 0.0;
        public double feederTempCelsius = 0.0;
        public boolean noteInIntake = false;
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(IndexerIOInputs inputs) {}

    default void setIntakeVoltage(double volts) {}

    default void setFeederVoltage(double volts) {}

    default void setFeederVelocity(double velocity) {}

    default void configureFeederPID(double p, double i, double d) {}

}
