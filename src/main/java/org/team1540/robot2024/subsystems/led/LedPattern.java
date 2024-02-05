package org.team1540.robot2024.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.function.Consumer;

public class LedPattern {
    private final Consumer<ZonedAddressableLEDBuffer> applier;
    public final boolean isDynamic;
    LedPattern(Consumer<ZonedAddressableLEDBuffer> applier, boolean isDynamic) {
        this.applier = applier;
        this.isDynamic = isDynamic;
    }

    void apply(ZonedAddressableLEDBuffer buffer) {
        this.applier.accept(buffer);
    }

    public static LedPattern solid(Color color) {
        return new LedPattern((buffer) -> {
            for (int i = 0; i<buffer.getLength();i++) {
                buffer.setLED(i, color);
            }
        }, false);
    }
    public static LedPattern alternating(Color a, Color b) {
        return new LedPattern((buffer) -> {
            for (int i = 0; i<buffer.getLength();i++) {
                buffer.setLED(i, i % 2 == 0 ? a : b);
            }
        }, false);
    }
}
