package org.team1540.robot2024.util.math;

public class JoystickUtils {
    public static double smartDeadzone(double rawInput, double deadzone) {
        double scalar = 1/(1-deadzone);
        if (rawInput > deadzone) {
            return (rawInput - deadzone)*scalar;
        }
        if (rawInput < -deadzone) {
            return (rawInput + deadzone)*scalar;
        }
        return 0;
    }

    public static double signedSquare(double value) {
        return value * Math.abs(value);
    }

    public static double squaredSmartDeadzone(double input, double deadzone) {
        return signedSquare(smartDeadzone(input, deadzone));
    }
}
