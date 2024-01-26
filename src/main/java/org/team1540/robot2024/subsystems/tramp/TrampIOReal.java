package org.team1540.robot2024.subsystems.tramp;

import edu.wpi.first.wpilibj.DigitalInput;

public class TrampIOReal implements TrampIO {
    DigitalInput beamBreak = new DigitalInput(0);
    public void updateInputs(TrampIOInputs inputs) {
        inputs.beamBreak = beamBreak.get();
    }
}
