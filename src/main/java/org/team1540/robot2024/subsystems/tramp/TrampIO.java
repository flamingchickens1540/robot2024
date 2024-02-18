package org.team1540.robot2024.subsystems.tramp;

import org.littletonrobotics.junction.AutoLog;


public interface TrampIO {
    @AutoLog
    class TrampIOInputs {
        public boolean noteInTramp = false;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }
    default void setVoltage(double volts) {}
    default void updateInputs(TrampIOInputs inputs) {}
}
