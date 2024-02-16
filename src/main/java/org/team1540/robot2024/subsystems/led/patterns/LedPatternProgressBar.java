package org.team1540.robot2024.subsystems.led.patterns;

import org.team1540.robot2024.subsystems.led.ZonedAddressableLEDBuffer;

import java.util.function.DoubleSupplier;

public class LedPatternProgressBar extends LedPattern {
    private final DoubleSupplier percent;

    public LedPatternProgressBar(DoubleSupplier percent) {
        super(true);
        this.percent = percent;
    }

    @Override
    public void apply(ZonedAddressableLEDBuffer buffer) {
        int barLength = (int) (buffer.getLength() * percent.getAsDouble());
        for (int i = 0; i < barLength; i++) {
            buffer.setHSV(i, 0, 200, 100); //This is semi dark red
        }
    }
}
