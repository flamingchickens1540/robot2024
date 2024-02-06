package org.team1540.robot2024.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

public abstract class LedPattern {
    private final boolean isDynamic;

    protected LedPattern(boolean isDynamic) {
        this.isDynamic = isDynamic;
    }

    boolean isDynamic() {
        return isDynamic;
    }

    abstract void apply(ZonedAddressableLEDBuffer buffer);

    protected static int getHue(Color color) {
        final int red = (int)color.red*255;
        final int green = (int)color.green*255;
        final int blue = (int)color.blue*255;
        float min = Math.min(Math.min(red, green), blue);
        float max = Math.max(Math.max(red, green), blue);

        if (min == max) {
            return 0;
        }

        float hue = 0f;
        if (max == red) {
            hue = (green - blue) / (max - min);

        } else if (max == green) {
            hue = 2f + (blue - red) / (max - min);

        } else {
            hue = 4f + (red - green) / (max - min);
        }

        hue = hue * 60;
        if (hue < 0) hue = hue + 360;

        return Math.round(hue);
    }
}
