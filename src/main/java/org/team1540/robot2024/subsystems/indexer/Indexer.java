package org.team1540.robot2024.subsystems.indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;

import java.util.function.BooleanSupplier;

import static org.team1540.robot2024.Constants.Indexer.*;


public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private double feederSetpointRPM = 0.0;

    private static boolean hasInstance = false;

    private Indexer(IndexerIO io) {
        if (hasInstance) throw new IllegalStateException("Instance of indexer already exists");
        hasInstance = true;
        this.io = io;
    }

    public static Indexer createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real indexer on simulated robot", false);
        }
        return new Indexer(new IndexerIOTalonFX());
    }

    public static Indexer createSim() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated indexer on real robot", false);
        }
        return new Indexer(new IndexerIOSim());
    }

    public static Indexer createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy indexer on real robot", false);
        }
        return new Indexer(new IndexerIO(){});
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        if (RobotState.isDisabled()){
            stopAll();
        }
    }

    public void setIntakePercent(double percent) {
        io.setIntakeVoltage(percent * 12.0);
    }

    public boolean isNoteStaged() {
        return inputs.noteInIndexer;
    }

    public BooleanSupplier checkNoteReached(NotePosition position) {
        return () -> getNoteState().ordinal() >= position.ordinal();
    }
    public NotePosition getNoteState() {
        if (inputs.noteInShooter) {
            return NotePosition.SHOOTER;
        }
        if (inputs.noteInIndexer) {
            return NotePosition.INDEXER;
        }
        if (inputs.noteInIntake) {
            return NotePosition.INTAKE;
        }
        return NotePosition.NONE;
    }

    public void setFeederVelocity(double setpointRPM) {
        feederSetpointRPM = setpointRPM;
        io.setFeederVelocity(setpointRPM);
    }

    public void setFeederPercent(double percent) {
        io.setFeederVoltage(12.0 * percent);
    }

    public boolean isFeederAtSetpoint() {
        return Math.abs(getFeederVelocityError()) < VELOCITY_ERR_TOLERANCE_RPM;
    }

    public void stopFeeder() {
        io.setFeederVoltage(0);
    }

    public void stopIntake() {
        io.setIntakeVoltage(0);
    }

    public void stopAll() {
        io.setIntakeVoltage(0);
        io.setFeederVoltage(0);
    }

    @AutoLogOutput(key = "Intake/Feeder/setpointRPM")
    public double getFeederVelocitySetpoint() {
        return feederSetpointRPM;
    }

    @AutoLogOutput(key = "Intake/Feeder/velocityErrorRPM")
    public double getFeederVelocityError() {
        return inputs.feederVelocityRPM - getFeederVelocitySetpoint();
    }

    public Command feedToAmp() {
        return Commands.runOnce(() -> io.setFeederVelocity(-600), this);
    }

    public Command feedToShooter() {
        return Commands.runOnce(() -> io.setFeederVelocity(1200), this);
    }

    public Command commandRunIntake(double percent) {
        return Commands.startEnd(
                () -> this.setIntakePercent(percent),
                () -> this.setIntakePercent(0),
                this
                );
    }

    public Command moveNoteTo(NotePosition position) {
        return Commands.startEnd(() -> {
            this.setIntakePercent(1);
            this.setFeederPercent(1);
        }, this::stopAll).until(this.checkNoteReached(NotePosition.SHOOTER));
    }

    public void setIntakeBrakeMode(boolean isBrake) {
        io.setIntakeBrakeMode(isBrake);
    }

    public double getIntakeVoltage() {
        return inputs.intakeVoltage;
    }

    public double getIntakeCurrent() {
        return inputs.intakeCurrentAmps;
    }

    public enum NotePosition {
        NONE,
        INTAKE,
        INDEXER,
        SHOOTER;
    }
}
