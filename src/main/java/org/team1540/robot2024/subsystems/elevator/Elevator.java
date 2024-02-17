package org.team1540.robot2024.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.util.MechanismVisualiser;
import org.team1540.robot2024.util.math.AverageFilter;

import static org.team1540.robot2024.Constants.Elevator.POS_ERR_TOLERANCE_METERS;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final AverageFilter positionFilter = new AverageFilter(10);
    private double setpointMeters;

    private static boolean hasInstance = false;

    private Elevator(ElevatorIO elevatorIO) {
        if (hasInstance) throw new IllegalStateException("Instance of elevator already exists");
        hasInstance = true;
        this.io = elevatorIO;
    }

    public static Elevator createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real elevator on simulated robot", false);
        }
        return new Elevator(new ElevatorIOTalonFX());
    }

    public static Elevator createSim() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated elevator on real robot", false);
        }
        return new Elevator(new ElevatorIOSim());
    }

    public static Elevator createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy elevator on real robot", false);
        }
        return new Elevator(new ElevatorIO(){});
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        MechanismVisualiser.setElevatorPosition(inputs.positionMeters);

        positionFilter.add(inputs.positionMeters);
    }

    public void setElevatorPosition(double positionMeters) {
        positionMeters = MathUtil.clamp(positionMeters, Constants.Elevator.MINIMUM_HEIGHT, Constants.Elevator.MAX_HEIGHT);
        setpointMeters = positionMeters;
        io.setSetpointMeters(setpointMeters);

        positionFilter.clear();
    }

    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpointMeters, positionFilter.getAverage(), POS_ERR_TOLERANCE_METERS);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    @AutoLogOutput
    public double getSetpoint() {
        return setpointMeters;
    }

    public double getPosition() {
        return inputs.positionMeters;
    }
}
