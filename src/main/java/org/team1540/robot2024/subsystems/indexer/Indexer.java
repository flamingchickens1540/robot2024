package org.team1540.robot2024.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.util.LoggedTunableNumber;

import static org.team1540.robot2024.Constants.Indexer.*;
import static org.team1540.robot2024.Constants.tuningMode;


public class Indexer extends SubsystemBase {
    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();
    private final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/kP", FEEDER_KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Indexer/kI", FEEDER_KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/kD", FEEDER_KD);


    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    public void periodic() {
        indexerIO.updateInputs(indexerInputs);
        Logger.processInputs("Indexer", indexerInputs);
        if (tuningMode) {
            if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
                indexerIO.configureFeederPID(kP.get(), kI.get(), kD.get());
            }
        }
    }

    public void setIntakePercent(double percent) {
        indexerIO.setIntakeVoltage(percent * 12.0);
    }

    public boolean isNotePresent() {
        return indexerInputs.noteInIntake;
    }

    public Command feedToAmp() {
        return Commands.runOnce(() -> indexerIO.setFeederVelocity(-600), this);
    }

    public Command feedToShooter() {
        return Commands.runOnce(() -> indexerIO.setFeederVelocity(1200), this);
    }


}
