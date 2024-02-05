package org.team1540.robot2024.subsystems.led;

public class LedPatternRainbow extends LedPattern {
    private final int speed;
    private int initialHue = 0;

    public LedPatternRainbow(int speed) {
        super(true);
        this.speed = speed;
    }

    @Override
    public boolean isDynamic() {
        return true;
    }

    @Override
    public void apply(ZonedAddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (initialHue + (i * 180 / buffer.getLength())) % 180;
            buffer.setHSV(i, hue, 255, 128);
        }
        initialHue += speed;
        initialHue %= 180;
    }

}
