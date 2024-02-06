package org.team1540.robot2024.subsystems.led;

public class LedPatternBreathing extends LedPattern{
    private final int speed;
    private int saturation;
    private boolean isReversed = false;
    public LedPatternBreathing(int speed) {
        super(true);
        this.speed = speed;
    }
    @Override
    void apply(ZonedAddressableLEDBuffer buffer) {
        if (saturation > 255) {
            saturation = 255;
            isReversed = !isReversed;
        }

        if (saturation < 0) {
            saturation = 0;
            isReversed = !isReversed;
        }
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, 0 , saturation, 255);
        }

        if (!isReversed) {
            saturation = saturation + speed;
        } else {
            saturation = saturation - speed;
        }
    }


}
