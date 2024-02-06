package org.team1540.robot2024.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

public class LedPatternWave extends LedPattern {
    private static final double degradation = 0.7;
    private static final int delay = 5;
    private final int hue;
    private int location = 0;
    private int ticker = 0;

    public LedPatternWave(Color color) {
        super(true);
        this.hue = getHue(color);
    }

    @Override
    void apply(ZonedAddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            int distance = Math.abs(location-i);
            if (distance > buffer.getLength()/2) {
                if (location > i) {
                    distance = buffer.getLength()-location + i;
                } else {
                    distance = buffer.getLength() -i + location;
                }
            }
            buffer.setHSV(i, hue, 255, (int) Math.max(255.0-distance*distance*degradation,0));
        }
        ticker++;
        if (ticker % delay == 0) {
            location++;
        }
        location %= buffer.getLength();
    }


}
