package org.team1540.robot2024.subsystems.led.patterns;

import edu.wpi.first.wpilibj.util.Color;
import org.team1540.robot2024.subsystems.led.ZonedAddressableLEDBuffer;

import java.util.function.BiConsumer;

public class SimpleLedPattern extends LedPattern{
    private final BiConsumer<ZonedAddressableLEDBuffer, Integer> applier;
    private SimpleLedPattern(BiConsumer<ZonedAddressableLEDBuffer, Integer> applier) {
        super(false);
        this.applier = applier;
    }

    public void apply(ZonedAddressableLEDBuffer buffer) {
        for (int i = 0; i < buffer.getLength(); i++) {
            this.applier.accept(buffer, i);
        }
    }

    public static LedPattern solid(Color color) {
        return new SimpleLedPattern((buffer, i) -> buffer.setLED(i, color));
    }
    public static LedPattern solid(String color) {
        return solid(new Color(color));
    }
    public static LedPattern alternating(Color... colors) {
        return alternating(1, colors);
    }
    public static LedPattern alternating(int groupSize, Color... colors) {
        final int colorCount = colors.length;
        return new SimpleLedPattern((buffer, i) -> buffer.setLED(i, colors[(i/groupSize)%colorCount]));
    }
    public static LedPattern blank() {
        return new LedPattern(false) {
            @Override
            public void apply(ZonedAddressableLEDBuffer buffer) {}
        };
    }
}
