package org.team1540.robot2024.subsystems.indexer;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.util.LoggedTunableNumber;
import static org.team1540.robot2024.Constants.Indexer.*;


public class Indexer extends SubsystemBase {
    private final IndexerIO indexerIO;
    private final IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();
    private final boolean TUNING = true;
    private final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/kP", FEEDER_KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Indexer/kI", FEEDER_KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/kD", FEEDER_KD);


    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

    public void periodic() {
        indexerIO.updateInputs(indexerInputs);
        Logger.processInputs("Indexer", indexerInputs);
        if (TUNING) {
            if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
                indexerIO.configurePID(kP.get(), kI.get(), kD.get());
            }
        }
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
