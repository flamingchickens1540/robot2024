package org.team1540.robot2024.subsystems.tramp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;

public class Tramp extends SubsystemBase {
    private final TrampIO io;
    private final TrampIOInputsAutoLogged inputs = new TrampIOInputsAutoLogged();
    private final PIDController positionalPID = new PIDController(1.8, 0,0);
    private boolean isClosedLoop = false;
    private static boolean hasInstance = false;

    private Tramp(TrampIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of tramp already exists");
        hasInstance = true;
        this.io = io;
    }

    public static Tramp createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real tramp on simulated robot", false);
        }
        return new Tramp(new TrampIOTalonFX());
    }

    public static Tramp createSim() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated tramp on real robot", false);
        }
        return new Tramp(new TrampIOSim());
    }

    public static Tramp createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy tramp on real robot", false);
        }
        return new Tramp(new TrampIO(){});
    }

    public void setPercent(double percentage) {
        isClosedLoop = false;
        io.setVoltage(12.0 * percentage);
    }
    public void setDistanceToGo(double distanceRots) {
        isClosedLoop = true;
        positionalPID.setSetpoint(distanceRots*Constants.Tramp.GEAR_RATIO+inputs.positionRots);
    }

    public boolean isNoteStaged() {
        return inputs.noteInTramp;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Tramp", inputs);
        if (isClosedLoop) {
            io.setVoltage(positionalPID.calculate(inputs.positionRots));
        }
    }

    public void stop() {
        this.setPercent(0);
    }

    public Command commandRun(double percent) {
        return Commands.startEnd(
                () -> this.setPercent(percent),
                this::stop,
                this
        );
    }
}
