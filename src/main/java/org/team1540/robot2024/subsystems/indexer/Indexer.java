package org.team1540.robot2024.subsystems.indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.util.LoggedTunableNumber;

import static org.team1540.robot2024.Constants.Indexer.*;


public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Indexer/kP", FEEDER_KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Indexer/kI", FEEDER_KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Indexer/kD", FEEDER_KD);

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
        return new Indexer(new IndexerIOSparkMax());
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

        if (Constants.isTuningMode()) {
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

    public void setFeederPercent(double percent) {
        io.setFeederVoltage(12.0 * percent);
    }

    public Command feedToAmp() {
        return Commands.runOnce(() -> io.setFeederVelocity(-600), this);
    }

    public Command feedToShooter() {
        return Commands.runOnce(() -> io.setFeederVelocity(1200), this);
    }
    
    public Command moveNoteOut() {
        return new FunctionalCommand(
                () -> setIntakePercent(-1),
                () -> {},
                (interrupted) -> stopIntake(),
                () -> !isNoteStaged(),
                this
        );
    }
    public boolean isFeederAtSetpoint() {
        return Math.abs(inputs.feederVelocityError) < VELOCITY_ERR_TOLERANCE_RPM;
//        return MathUtil.isNear(inputs.setpointRPM, inputs.feederVelocityRPM, VELOCITY_ERR_TOLERANCE_RPM);
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

    public Command commandRunIntake(double percent) {
        return Commands.startEnd(
                () -> this.setIntakePercent(percent),
                () -> this.setIntakePercent(0),
                this
                );
    }

    public Command setIntakeAndFeeder(double intakePercent, double feederVelocity) {
        return Commands.startEnd(
                () -> {setIntakePercent(intakePercent); setFeederVelocity(feederVelocity);},
                () -> {setIntakePercent(0); setFeederVelocity(0);},
                this
        );
    }

}
