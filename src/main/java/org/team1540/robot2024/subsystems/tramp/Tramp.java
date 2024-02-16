package org.team1540.robot2024.subsystems.tramp;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Tramp extends SubsystemBase {
    private final TrampIO io;
    private final TrampIOInputsAutoLogged inputs = new TrampIOInputsAutoLogged();

    private static boolean hasInstance = false;

    private Tramp(TrampIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of tramp already exists");
        hasInstance = true;
        this.io = io;
    }

    public static Tramp createReal() {
        return new Tramp(new TrampIOSparkMax());
    }

    public static Tramp createSim() {
        return new Tramp(new TrampIOSim());
    }

    public static Tramp createDummy() {
        return new Tramp(new TrampIO(){});
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
