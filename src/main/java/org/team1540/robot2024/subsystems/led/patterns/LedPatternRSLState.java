package org.team1540.robot2024.subsystems.led.patterns;

import edu.wpi.first.wpilibj.RobotController;
import org.team1540.robot2024.subsystems.led.ZonedAddressableLEDBuffer;

public class LedPatternRSLState extends LedPattern {
    private final LedPattern a;
    private final LedPattern b;
    public LedPatternRSLState(LedPattern a, LedPattern b) {
        super(true);
        this.a = a;
        this.b = b;
    }

    @Override
    public void apply(ZonedAddressableLEDBuffer buffer) {
        if (RobotController.getRSLState()) {
            a.apply(buffer);
        } else {
            b.apply(buffer);
        }
    }
}