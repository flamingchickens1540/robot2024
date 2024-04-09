package org.team1540.robot2024.subsystems.led.patterns;

import org.team1540.robot2024.subsystems.led.ZonedAddressableLEDBuffer;

import edu.wpi.first.wpilibj.util.Color;
import java.util.function.DoubleSupplier;

public class LedPatternProgressBar extends LedPattern {
    private final DoubleSupplier percent;
    private final Color color;
    private final int maxLength;

    public LedPatternProgressBar(DoubleSupplier percent, String hex, int maxLength) {
        super(true);
        this.percent = percent;
        this.color = new Color(hex);
        this.maxLength = maxLength;
    }

    @Override
    public void apply(ZonedAddressableLEDBuffer buffer) {
        int barLength = (int) (maxLength * percent.getAsDouble());
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
        for (int i = 0; i < barLength; i++) {
            buffer.setLED(i, color);
        }
    }
}
