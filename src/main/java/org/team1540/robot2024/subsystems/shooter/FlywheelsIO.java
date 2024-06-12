package org.team1540.robot2024.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
    @AutoLog
    class FlywheelsIOInputs {
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double leftVelocityRPM = 0.0;
        public double leftTempCelsius = 0.0;
        public boolean leftMotorConnected = true;

        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        public double rightVelocityRPM = 0.0;
        public double rightTempCelsius = 0.0;
        public boolean rightMotorConnected = true;
    }

    /**
     * Updates the set of loggable inputs
     */
    default void updateInputs(FlywheelsIOInputs inputs) {}

    /**
     * Runs open loop at the specified voltages
     */
    default void setVoltage(double leftVolts, double rightVolts) {}


    /**
     * Runs closed loop at the specified RPMs
     */
    default void setSpeeds(double leftRPM, double rightRPM) {}

    /**
     * Configures the PID controller
     */
    default void configPID(double kP, double kI, double kD, double kV) {}
}
