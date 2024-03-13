package org.team1540.robot2024.subsystems.led.patterns;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import org.team1540.robot2024.subsystems.led.ZonedAddressableLEDBuffer;

public class LedPatternRSLState extends LedPattern {
    private final LedPattern a;
    private final LedPattern b;
    public LedPatternRSLState(LedPattern a, LedPattern b) {
        super(true);
        this.a = a;
        this.b = b;
    }

    public LedPatternRSLState(LedPattern a) {
        this(a, SimpleLedPattern.solid(Color.kBlack));
    }
    public LedPatternRSLState(Color a) {
        this(SimpleLedPattern.solid(a), SimpleLedPattern.solid(Color.kBlack));
    }

    @Override
    public void apply(ZonedAddressableLEDBuffer buffer) {
        if (RobotController.getRSLState()) {
            a.apply(buffer);
        } else {
            b.apply(buffer);
        }
    }

    public static LedPattern matchingColors() {
        return new LedPatternRSLState(new Color("#FF1A00"));
    }
}