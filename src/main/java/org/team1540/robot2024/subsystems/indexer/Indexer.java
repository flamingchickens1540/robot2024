package org.team1540.robot2024.subsystems.indexer;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;


public class Indexer extends SubsystemBase {
    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    public void periodic() {
        indexerIO.updateInputs(indexerInputs);
        Logger.processInputs("Indexer", indexerInputs);
    }

    public void setIntakePercent(double percent) {
        indexerIO.setIntakeVoltage(percent * 12.0);
    }

    public boolean noteInIndexer() {
        return indexerInputs.noteInIntake;
    }

    public Command feedToAmp() {
        return Commands.sequence(
                Commands.runOnce(() -> indexerIO.setFeederVelocity(-600), this),
                Commands.waitSeconds(5),
                Commands.runOnce(() -> indexerIO.setFeederVelocity(0), this)
        );
    }

    public Command feedToShooter() {
        return Commands.sequence(
                Commands.runOnce(() -> indexerIO.setFeederVelocity(1200), this),
                Commands.waitSeconds(1),
                Commands.runOnce(() -> indexerIO.setFeederVelocity(0), this)
        );
    }


}
