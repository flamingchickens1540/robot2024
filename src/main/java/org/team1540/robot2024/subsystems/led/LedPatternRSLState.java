package org.team1540.robot2024.subsystems.led;

import edu.wpi.first.wpilibj.RobotController;

public class LedPatternRSLState extends LedPattern {
    private LedPattern pattern1;
    private LedPattern pattern2;
    public LedPatternRSLState(LedPattern pattern1, LedPattern pattern2) {
        super(true);
        this.pattern1 = pattern1;
        this.pattern2 = pattern2;
    }

    @Override
    void apply(ZonedAddressableLEDBuffer buffer) {
        if (RobotController.getRSLState()) {
            pattern1.apply(buffer);
        } else {
            pattern2.apply(buffer);
        }
    }
}