package org.team1540.robot2024.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.subsystems.led.patterns.LedPattern;


import static org.team1540.robot2024.Constants.LED_STRIP_PORT_PWM;

public class Leds extends SubsystemBase {
    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_STRIP_LENGTH);
    private final AddressableLED strip = new AddressableLED(LED_STRIP_PORT_PWM);
    private final ZonedAddressableLEDBuffer[] buffers = new ZonedAddressableLEDBuffer[ZONE_COUNT];
    private final LedTriager[] patterns = new LedTriager[ZONE_COUNT];

    public Leds() {
        strip.setLength(ledBuffer.getLength());
        strip.setData(ledBuffer);
        strip.start();

        buffers[Zone.ZONE1.ordinal()] = new ZonedAddressableLEDBuffer(ledBuffer, 1, 40, false);
        buffers[Zone.ZONE2.ordinal()] = new ZonedAddressableLEDBuffer(ledBuffer, 40, 80, false);
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
        setPattern(zone, pattern, PatternCriticality.INFO);
    }

    public void clearPattern(Zone zone, PatternCriticality criticality) {
        patterns[zone.ordinal()].clearPattern(criticality);
    }

    public void setFatalPattern(LedPattern pattern) {
        System.out.println("Setting criticality");
        for (int i = 0; i<ZONE_COUNT;i++) {
            patterns[i].addPattern(pattern, PatternCriticality.FATAL);
            pattern.setLength(buffers[i].getLength());
        }
    }

    public void clearFatalPattern() {
        System.out.println("Clearing criticality");
        for (int i = 0; i<ZONE_COUNT;i++) {
            patterns[i].clearPattern(PatternCriticality.FATAL);
        }

    }


    private static final int ZONE_COUNT=Zone.values().length;
    public enum Zone {
        ZONE1,
        ZONE2;
    }
    static final int CRITICALITY_COUNT=PatternCriticality.values().length;
    public enum PatternCriticality {
        DECORATION,
        INFO,
        CRITICAL,
        FATAL
    }

}
