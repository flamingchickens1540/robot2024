package org.team1540.robot2024.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.util.Alert;
import org.team1540.robot2024.util.LoggedTunableNumber;
import org.team1540.robot2024.util.MechanismVisualiser;
import org.team1540.robot2024.util.math.AverageFilter;

import static org.team1540.robot2024.Constants.Elevator.*;
import static org.team1540.robot2024.Constants.Elevator.POS_ERR_TOLERANCE_METERS;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final AverageFilter positionFilter = new AverageFilter(10);
    private double setpointMeters;

    private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", KP);
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", KI);
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", KD);

    private final Alert leadMotorDisconnected =
            new Alert("Elevator lead motor disconnected!", Alert.AlertType.WARNING);
    private final Alert followMotorDisconnected =
            new Alert("Elevator follower motor disconnected!", Alert.AlertType.WARNING);

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

        if (RobotState.isDisabled()) stop();

        positionFilter.add(inputs.positionMeters);

        if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.configPID(kP.get(), kI.get(), kD.get());
        }
        if(getPosition() > 0.31){
            setFlipper(false);
        }

        leadMotorDisconnected.set(!inputs.leadMotorConnected);
        followMotorDisconnected.set(!inputs.followMotorConnected);
    }

    public void setElevatorPosition(double positionMeters) {
        positionMeters = MathUtil.clamp(positionMeters, MINIMUM_HEIGHT, MAX_HEIGHT);
        setpointMeters = positionMeters;
        io.setSetpointMeters(setpointMeters);

        positionFilter.clear();
    }

    public void setFlipper(boolean flipped){
        io.setFlipper(flipped);
    }

    @AutoLogOutput(key = "Elevator/isAtSetpoint")
    public boolean isAtSetpoint() {
        return MathUtil.isNear(setpointMeters, positionFilter.getAverage(), POS_ERR_TOLERANCE_METERS) || (inputs.atLowerLimit && setpointMeters <= 0);
    }

    public void setVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stop() {
        io.setVoltage(0.0);
    }

    @AutoLogOutput(key = "Elevator/setpoint")
    public double getSetpoint() {
        return setpointMeters;
    }

    public double getPosition() {
        return inputs.positionMeters;
    }

    public double getVelocity() {
        return inputs.velocityMPS;
    }

    public void setBrakeMode(boolean isBrakeMode) {
        io.setBrakeMode(isBrakeMode);
    }

    public boolean getUpperLimit(){
        return inputs.atUpperLimit;
    }

    public boolean getLowerLimit(){
        return inputs.atLowerLimit;
    }

    public void holdPosition() {
        setElevatorPosition(inputs.positionMeters);
    }
}
