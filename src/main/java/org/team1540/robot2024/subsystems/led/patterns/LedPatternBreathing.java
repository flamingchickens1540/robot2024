package org.team1540.robot2024.subsystems.led.patterns;

import edu.wpi.first.wpilibj.util.Color;
import org.team1540.robot2024.subsystems.led.ZonedAddressableLEDBuffer;

public class LedPatternBreathing extends LedPattern {
    private final int speed;
    private final int hue;
    private final int saturation;
    private int value = min;
    private boolean isReversed = false;
    private static final int max = 255;
    private static final int min = 60;

    public LedPatternBreathing(int speed, Color color) {
        super(true);
        this.speed = speed;
        int[] hsv = getHSV(color);
        this.hue = hsv[0];
        this.saturation = hsv[1];
    }
    public LedPatternBreathing(int speed, String color) {
        this(speed, new Color(color));
    }
    public LedPatternBreathing(String color) {
        this(3, new Color(color));
    }


    @Override
    public void apply(ZonedAddressableLEDBuffer buffer) {
        if (value > max) {
            value = max;
            isReversed = !isReversed;
        }

        if (value < min) {
            value = min;
            isReversed = !isReversed;
        }
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, hue, saturation, value);
        }

        if (!isReversed) {
            value = value + speed;
        } else {
            value = value - speed;
        }
    }


}
