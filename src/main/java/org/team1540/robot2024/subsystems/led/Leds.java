package org.team1540.robot2024.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2024.Constants;


import static org.team1540.robot2024.Constants.LED_STRIP_PORT_PWM;

public class Leds extends SubsystemBase {

    private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_STRIP_LENGTH);
    private final AddressableLED strip = new AddressableLED(LED_STRIP_PORT_PWM);
    private final ZonedAddressableLEDBuffer[] buffers = new ZonedAddressableLEDBuffer[ZONE_COUNT];
    private final LedPattern[] patterns = new LedPattern[ZONE_COUNT];

    public Leds() {
        strip.setLength(ledBuffer.getLength());
        strip.setData(ledBuffer);
        strip.start();

        buffers[Zone.ZONE1.ordinal()] = new ZonedAddressableLEDBuffer(ledBuffer, 0, 40, false);
        buffers[Zone.ZONE2.ordinal()] = new ZonedAddressableLEDBuffer(ledBuffer, 40, 80, false);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ZONE_COUNT;i++) {
            if (patterns[i].isDynamic()) {
                patterns[i].apply(buffers[i]);
            }
        }
        strip.setData(ledBuffer);
    }

    public void setPattern(Zone zone, LedPattern pattern) {
        patterns[zone.ordinal()] = pattern;
        if (!pattern.isDynamic()) {
            pattern.apply(buffers[zone.ordinal()]);
        }
    }


    private static final int ZONE_COUNT=Zone.values().length;
    public enum Zone {
        ZONE1,
        ZONE2;
    }
}
