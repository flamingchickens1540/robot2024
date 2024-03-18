package org.team1540.robot2024.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2024.subsystems.led.patterns.LedPattern;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Leds.*;

public class Leds extends SubsystemBase {
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LED_STRIP_LENGTH);
    private final AddressableLED strip = new AddressableLED(LED_STRIP_PORT_PWM);
    private final ZonedAddressableLEDBuffer[] buffers = new ZonedAddressableLEDBuffer[ZONE_COUNT];
    private final LedTriager[] patterns = new LedTriager[ZONE_COUNT];

    public Leds() {
        strip.setLength(ledBuffer.getLength());
        strip.setData(ledBuffer);
        strip.start();

        buffers[Zone.ELEVATOR_BACK.ordinal()] = new ZonedAddressableLEDBuffer(ledBuffer, 1, 41, false);
        for (int i = 0; i < ZONE_COUNT;i++) {
            patterns[i] = new LedTriager();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ZONE_COUNT;i++) {
            if (patterns[i].shouldRefresh()) {
                patterns[i].getPattern().apply(buffers[i]);
            }
        }
        strip.setData(ledBuffer);
    }

    public void setPattern(Zone zone, LedPattern pattern, PatternCriticality criticality) {
        patterns[zone.ordinal()].addPattern(pattern, criticality);
        pattern.setLength(buffers[zone.ordinal()].getLength());
    }

    public void setPattern(Zone zone, LedPattern pattern) {
        setPattern(zone, pattern, PatternCriticality.NORMAL);
    }

    public void clearPattern(Zone zone, PatternCriticality criticality) {
        patterns[zone.ordinal()].clearPattern(criticality);
    }

    public void setPatternAll(Supplier<LedPattern> patternSupplier, PatternCriticality criticality) {
        for (int i = 0; i<ZONE_COUNT;i++) {
            LedPattern pattern = patternSupplier.get();
            patterns[i].addPattern(pattern, criticality);
            pattern.setLength(buffers[i].getLength());
        }
    }

    public void clearPatternAll(PatternCriticality criticality) {
        for (int i = 0; i<ZONE_COUNT;i++) {
            patterns[i].clearPattern(criticality);
        }
    }


    private static final int ZONE_COUNT=Zone.values().length;
    public enum Zone {
        ELEVATOR_BACK,
    }
    static final int CRITICALITY_COUNT=PatternCriticality.values().length;
    public enum PatternCriticality {
        LOWEST,
        NORMAL,
        MID,
        HIGH,
        HAS_INTAKE,
        DRIVER_LOCK,
        EXTREME
    }

    public Command commandShowPattern(LedPattern pattern, Leds.PatternCriticality criticality) {
        return Commands.startEnd(
                () -> this.setPattern(Leds.Zone.ELEVATOR_BACK, pattern, criticality),
                () -> this.clearPattern(Leds.Zone.ELEVATOR_BACK, criticality)
        );
    }

}
