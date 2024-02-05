package org.team1540.robot2024.subsystems.led;

import static java.lang.Math.round;

public class LedPatternRainbow extends LedPattern {
    private final int speed;
    int initialHue = 0;
    public LedPatternRainbow(int speed) {
        super(null, true);
        this.speed = speed;
    }

    @Override
    void apply(ZonedAddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (initialHue + (i * 180 / buffer.getLength())) % 180;
            buffer.setHSV(i, hue, 255, 128);
        }
        initialHue += speed;
        initialHue %= 180;
    }

}
