package org.team1540.robot2024.subsystems.led.patterns;

import edu.wpi.first.wpilibj.util.Color;
import org.team1540.robot2024.subsystems.led.ZonedAddressableLEDBuffer;

public class LedPatternBreathing extends LedPattern {
    private final int speed;
    private final int hue;
    private int saturation;
    private boolean isReversed = false;

    public LedPatternBreathing(int speed, Color color) {
        super(true);
        this.speed = speed;
        this.hue = getHSV(color)[0];
    }

    @Override
    public void apply(ZonedAddressableLEDBuffer buffer) {
        if (saturation > 255) {
            saturation = 255;
            isReversed = !isReversed;
        }

        if (saturation < 0) {
            saturation = 0;
            isReversed = !isReversed;
        }
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, hue, saturation, 255);
        }

        if (!isReversed) {
            saturation = saturation + speed;
        } else {
            saturation = saturation - speed;
        }
    }


}
