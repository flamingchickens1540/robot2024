package org.team1540.robot2024.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {
    @AutoLog
    class ShooterPivotIOInputs {
        public Rotation2d position = new Rotation2d();
        public Rotation2d absolutePosition = new Rotation2d();
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public boolean isAtForwardLimit = false;
        public boolean isAtReverseLimit = false;
    }

    /**
     * Updates the set of loggable inputs
     */
    default void updateInputs(ShooterPivotIOInputs inputs) {}

    /**
     * Runs closed loop to the specified position
     */
    default void setPosition(Rotation2d position) {}

    /**
     * Runs open loop at the specified voltage
     */
    default void setVoltage(double volts) {}

    /**
     * Sets the neutral output mode
     */
    default void setBrakeMode(boolean isBrakeMode) {}

    /**
     * Configures the PID controller
     */
    default void configPID(double kP, double kI, double kD, double kG) {}

    default void setEncoderPosition(double rots) {}
}
