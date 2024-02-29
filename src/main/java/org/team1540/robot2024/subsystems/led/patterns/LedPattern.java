package org.team1540.robot2024.subsystems.led.patterns;

import edu.wpi.first.wpilibj.util.Color;
import org.team1540.robot2024.subsystems.led.ZonedAddressableLEDBuffer;

public abstract class LedPattern {
    private final boolean isDynamic;

    protected LedPattern(boolean isDynamic) {
        this.isDynamic = isDynamic;
    }

    public final boolean isDynamic() {
        return isDynamic;
    }

    public abstract void apply(ZonedAddressableLEDBuffer buffer);
    public void setLength(int length) {}

    protected static int getHue(Color color) {
        final double red = color.red;
        final double green = color.green;
        final double blue = color.blue;
        double min = Math.min(Math.min(red, green), blue);
        double max = Math.max(Math.max(red, green), blue);

        if (min == max) {
            return 0;
        }

        double hue;
        if (max == red) {
            hue = (green - blue) / (max - min);

        } else if (max == green) {
            hue = 2f + (blue - red) / (max - min);

        } else {
            hue = 4f + (red - green) / (max - min);
        }

        hue = -hue * 60;
        if (hue > 0) hue = hue - 360;

        return (int) Math.round(hue);
    }
}
