package org.team1540.robot2024.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    class IndexerIOInputs {
        public double intakeVoltage = 0.0;
        public double intakeCurrent = 0.0;
        public double intakeVelocityRPM = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(IndexerIOInputs inputs) {}

    default void setIntakeVoltage(double volts) {}

    default void setIntakeSpeed(double speed) {}



}
