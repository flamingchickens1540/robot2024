package org.team1540.robot2024.subsystems.led;

import org.team1540.robot2024.subsystems.led.patterns.LedPattern;
import org.team1540.robot2024.subsystems.led.patterns.LedPatternRainbow;


public class LedTriager {
    private final LedPattern[] patterns = new LedPattern[Leds.LEVEL_COUNT];
    private final LedPattern defaultPattern = new LedPatternRainbow(1);
    private boolean isNew = true;
    public LedPattern getPattern() {
        for (int i = patterns.length -1; i >= 0; i--) {
            if (patterns[i] != null) {
                return patterns[i];
            }
        }
        return defaultPattern;
    }

    public boolean shouldRefresh() {
        final boolean val = isNew || getPattern().isDynamic();
        isNew = false;
        return val;
    }

    public void clearPattern(Leds.PatternLevel criticality) {
        patterns[criticality.ordinal()] = null;
        isNew = true;
    }

    public boolean addPattern(LedPattern pattern, Leds.PatternLevel criticality) {
        patterns[criticality.ordinal()] = pattern;
        isNew = true;
        return getPattern() == pattern;
    }

}
