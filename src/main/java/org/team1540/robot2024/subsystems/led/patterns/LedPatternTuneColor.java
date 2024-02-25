package org.team1540.robot2024.subsystems.led.patterns;

import org.team1540.robot2024.subsystems.led.ZonedAddressableLEDBuffer;
import org.team1540.robot2024.util.LoggedTunableNumber;

public class LedPatternTuneColor extends LedPattern {
    LoggedTunableNumber r = new LoggedTunableNumber("led/r", 0);
    LoggedTunableNumber g = new LoggedTunableNumber("led/g", 0);
    LoggedTunableNumber b = new LoggedTunableNumber("led/b", 0);
    public LedPatternTuneColor() {
        super(true);
    }

    @Override
    public void apply(ZonedAddressableLEDBuffer buffer) {
        if (r.hasChanged(hashCode())||g.hasChanged(hashCode())||b.hasChanged(hashCode())) {
            int rVal = (int) r.get();
            int gVal = (int) g.get();
            int bVal = (int) b.get();
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(i, rVal, gVal, bVal);
            }
        }
    }
}
