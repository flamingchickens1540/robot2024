package org.team1540.robot2024.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

import java.util.function.BiConsumer;

public class LedPattern {
    private final BiConsumer<ZonedAddressableLEDBuffer, Integer> applier;
    public final boolean isDynamic;
    private LedPattern(BiConsumer<ZonedAddressableLEDBuffer, Integer> applier, boolean isDynamic) {
        this.applier = applier;
        this.isDynamic = isDynamic;
    }
    protected LedPattern(boolean isDynamic) {
        this.applier = null;
        this.isDynamic = isDynamic;
    }

    void apply(ZonedAddressableLEDBuffer buffer) {
        assert this.applier != null;
        for (int i = 0; i < buffer.getLength(); i++) {
            this.applier.accept(buffer, i);
        }
    }

    public static LedPattern solid(Color color) {
        return new LedPattern((buffer, i) -> buffer.setLED(i, color), false);
    }
    public static LedPattern alternating(Color a, Color b) {
        return new LedPattern((buffer, i) -> buffer.setLED(i, i % 2 == 0 ? a : b), false);
    }
}
