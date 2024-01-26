package org.team1540.robot2024.subsystems.tramp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Tramp extends SubsystemBase {
    private final TrampIO io;
    private final TrampIOInputsAutoLogged inputs = new TrampIOInputsAutoLogged();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Tramp(TrampIO io) {
        this.io = io;

    }
}
