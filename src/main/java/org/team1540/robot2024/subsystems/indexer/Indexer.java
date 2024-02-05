package org.team1540.robot2024.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.util.LoggedTunableNumber;

import static org.team1540.robot2024.Constants.Indexer.*;
import static org.team1540.robot2024.Constants.tuningMode;


public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/kP", FEEDER_KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Indexer/kI", FEEDER_KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/kD", FEEDER_KD);


    public Indexer(IndexerIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        if (tuningMode) {
            if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
                io.configureFeederPID(kP.get(), kI.get(), kD.get());
            }
        }
    }

    public void setIntakePercent(double percent) {
        io.setIntakeVoltage(percent * 12.0);
    }

    public boolean isNoteStaged() {
        return inputs.noteInIntake;
    }

    public void setFeederVelocity(double setpointRPM) {
        io.setFeederVelocity(setpointRPM);
    }

    public Command feedToAmp() {
        return Commands.runOnce(() -> io.setFeederVelocity(-600), this);
    }

    public Command feedToShooter() {
        return Commands.runOnce(() -> io.setFeederVelocity(1200), this);
    }

    // TODO: Add method to check if feeder is spun up

    public void stopFeeder() {
        io.setFeederVoltage(0);
    }
    public void stopIntake() {
        io.setIntakeVoltage(0);
    }
}
