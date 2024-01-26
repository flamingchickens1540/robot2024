package org.team1540.robot2024.subsystems.indexer;

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

    public void setIntakeSpeed(double speed) {
        indexerIO.setIntakeSpeed(speed);
    }

    public void setIntakeVoltage(double volts) {
        indexerIO.setIntakeVoltage(volts);
    }



}
