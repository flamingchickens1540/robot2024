package org.team1540.robot2024.util;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class LoggedTunableNumber extends LoggedDashboardNumber {
    private double lastHasChangedValue;

    public LoggedTunableNumber(String key, double defaultValue) {
        super(key, defaultValue);
        lastHasChangedValue = defaultValue;
    }

    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue != lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            return true;
        }
        return false;
    }
}