package org.team1540.robot2024.subsystems.tramp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Tramp extends SubsystemBase {
    private final TrampIO io;
    private final TrampIOInputsAutoLogged inputs = new TrampIOInputsAutoLogged();

    public Tramp(TrampIO io) {
        this.io = io;
    }

    public void setPercent(double percentage) {
        io.setVoltage(12.0 * percentage);
    }

    public boolean isNoteStaged() {
        return inputs.breamBreakTripped;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Tramp", inputs);
    }

    public void stop() {
        io.setVoltage(0);
    }

}
