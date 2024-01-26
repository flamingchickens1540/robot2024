package org.team1540.robot2024.subsystems.tramp;

import org.littletonrobotics.junction.AutoLog;


public interface TrampIO {
    @AutoLog
    class TrampIOInputs {
        boolean beamBreak = false;
        double motorVelocity;
    }

    default void updateInputs(TrampIOInputs inputs) {
    }
}
