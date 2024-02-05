package org.team1540.robot2024.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

import java.util.function.BiConsumer;

public class SimpleLedPattern extends LedPattern{
    private final BiConsumer<ZonedAddressableLEDBuffer, Integer> applier;
    private SimpleLedPattern(BiConsumer<ZonedAddressableLEDBuffer, Integer> applier) {
        super(false);
        this.applier = applier;
    }

    void apply(ZonedAddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            this.applier.accept(buffer, i);
        }
    }

    public static LedPattern solid(Color color) {
        return new SimpleLedPattern((buffer, i) -> buffer.setLED(i, color));
    }
    public static LedPattern alternating(Color... colors) {
        final int colorCount = colors.length;
        return new SimpleLedPattern((buffer, i) -> buffer.setLED(i, colors[i%colorCount]));
    }
    public static LedPattern alternating(int groupSize, Color... colors) {
        final int colorCount = colors.length;
        return new SimpleLedPattern((buffer, i) -> buffer.setLED(i, colors[(i/groupSize)%colorCount]));
    }
}
