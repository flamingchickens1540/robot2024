package org.team1540.robot2024.subsystems.led.patterns;

import org.team1540.robot2024.subsystems.led.ZonedAddressableLEDBuffer;

import edu.wpi.first.wpilibj.util.Color;
import java.util.function.DoubleSupplier;

public class LedPatternProgressBar extends LedPattern {
    private final DoubleSupplier percent;
    private final int hue;

    public LedPatternProgressBar(DoubleSupplier percent, Color color) {
        super(true);
        this.percent = percent;
        this.hue = getHue(color);
    }
    @Override
    public void apply(ZonedAddressableLEDBuffer buffer) {
        int barLength = (int) (buffer.getLength() * percent.getAsDouble());
        for (int i =0; i < buffer.getLength(); i++) {
            buffer.setHSV(i,0,0,0);
        }
        for (int i = 0; i < barLength; i++) {
            buffer.setHSV(i, hue, 255, 255);
        }
    }
}
