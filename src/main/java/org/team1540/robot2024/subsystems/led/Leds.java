package org.team1540.robot2024.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2024.Robot;
import org.team1540.robot2024.subsystems.led.patterns.LedPattern;


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

        buffers[Zone.MAIN.ordinal()] = new ZonedAddressableLEDBuffer(ledBuffer, 0, 41, false);
        buffers[Zone.TOP.ordinal()] = new ZonedAddressableLEDBuffer(ledBuffer, 32, 41, false);
        for (int i = 0; i < ZONE_COUNT;i++) {
            patterns[i] = new LedTriager();
        }
    }

    @Override
    public void periodic() {
        try {
            for (int i = 0; i < ZONE_COUNT; i++) {
                if (patterns[i].shouldRefresh()) {
                    patterns[i].getPattern().apply(buffers[i]);
                }
            }
            strip.setData(ledBuffer);
        } catch (Exception e) {
            if (Robot.isReal()) {
                DriverStation.reportWarning("Error in LEDs periodic: "+e+": "+e.getMessage(), e.getStackTrace());
            } else {
                throw e;
            }
        }
    }

    public void setPattern(Zone zone, LedPattern pattern, PatternLevel priority) {
        patterns[zone.ordinal()].addPattern(pattern, priority);
        pattern.setLength(buffers[zone.ordinal()].getLength());
    }

    public void setPattern(Zone zone, LedPattern pattern) {
        setPattern(zone, pattern, PatternLevel.DEFAULT);
    }

    public void clearPattern(Zone zone, PatternLevel priority) {
        patterns[zone.ordinal()].clearPattern(priority);
    }




    private static final int ZONE_COUNT=Zone.values().length;
    public enum Zone {
        MAIN,
        TOP
    }
    static final int LEVEL_COUNT = PatternLevel.values().length;
    public enum PatternLevel {
        DEFAULT,
        TRAMP_STATE,
        INTAKE_STATE,
        COAST_STATE,
        DRIVER_LOCK,
    }

    public Command commandShowPattern(LedPattern pattern, PatternLevel priority) {
        return Commands.startEnd(
                () -> this.setPattern(Leds.Zone.MAIN, pattern, priority),
                () -> this.clearPattern(Leds.Zone.MAIN, priority)
        ).ignoringDisable(true);
    }

    public Command commandShowIntakePattern(LedPattern pattern) {
        return Commands.startEnd(
                () -> {
                    this.setPattern(Zone.MAIN, pattern, PatternLevel.INTAKE_STATE);
                    this.setPattern(Zone.TOP, pattern, PatternLevel.INTAKE_STATE);
                },
                () -> {
                    this.clearPattern(Zone.MAIN,  PatternLevel.INTAKE_STATE);
                    this.clearPattern(Zone.TOP,  PatternLevel.INTAKE_STATE);
                }
        ).ignoringDisable(true);
    }

    public Command commandShowTrampPattern(LedPattern pattern) {
        return Commands.startEnd(
                () -> {
                    this.setPattern(Zone.MAIN, pattern, PatternLevel.TRAMP_STATE);
                    this.setPattern(Zone.TOP, pattern, PatternLevel.TRAMP_STATE);
                },
                () -> {
                    this.clearPattern(Zone.MAIN,  PatternLevel.TRAMP_STATE);
                    this.clearPattern(Zone.TOP,  PatternLevel.TRAMP_STATE);
                }
        ).ignoringDisable(true);
    }
    public Command commandSet(LedPattern pattern, PatternLevel priority) {
        return Commands.runOnce(() -> this.setPattern(Leds.Zone.MAIN, pattern, priority)).ignoringDisable(true);
    }
    public Command commandClear(PatternLevel priority) {
        return Commands.runOnce(() -> this.clearPattern(Leds.Zone.MAIN, priority)).ignoringDisable(true);
    }

    public void setPatternAll(LedPattern pattern, PatternLevel level) {
        this.setPattern(Zone.MAIN, pattern, level);
        this.setPattern(Zone.TOP, pattern, level);
    }
    public void clearPatternAll(PatternLevel level) {
        this.clearPattern(Zone.MAIN, level);
        this.clearPattern(Zone.TOP, level);
    }

}
