package org.team1540.robot2024.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.Constants;
import org.team1540.robot2024.util.LoggedTunableNumber;
import org.team1540.robot2024.util.MechanismVisualiser;
import org.team1540.robot2024.util.math.AverageFilter;

import java.util.function.Supplier;

import static org.team1540.robot2024.Constants.Shooter.Flywheels;
import static org.team1540.robot2024.Constants.Shooter.Pivot;

public class Shooter extends SubsystemBase {
    private final ShooterPivotIO pivotIO;
    private final ShooterPivotIOInputsAutoLogged pivotInputs = new ShooterPivotIOInputsAutoLogged();

    private final FlywheelsIO flywheelsIO;
    private final FlywheelsIOInputsAutoLogged flywheelInputs = new FlywheelsIOInputsAutoLogged();

    private final AverageFilter leftSpeedFilter = new AverageFilter(20); // Units: RPM
    private final AverageFilter rightSpeedFilter = new AverageFilter(20); // Units: RPM
    private final AverageFilter pivotPositionFilter = new AverageFilter(10); // Units: rotations


    private double leftFlywheelSetpointRPM;
    private double rightFlywheelSetpointRPM;
    private Rotation2d pivotSetpoint = new Rotation2d();

    private final LoggedTunableNumber flywheelsKP = new LoggedTunableNumber("Shooter/Flywheels/kP", Flywheels.KP);
    private final LoggedTunableNumber flywheelsKI = new LoggedTunableNumber("Shooter/Flywheels/kI", Flywheels.KI);
    private final LoggedTunableNumber flywheelsKD = new LoggedTunableNumber("Shooter/Flywheels/kD", Flywheels.KD);

    private final LoggedTunableNumber pivotKP = new LoggedTunableNumber("Shooter/Pivot/kP", Pivot.KP);
    private final LoggedTunableNumber pivotKI = new LoggedTunableNumber("Shooter/Pivot/kI", Pivot.KI);
    private final LoggedTunableNumber pivotKD = new LoggedTunableNumber("Shooter/Pivot/kD", Pivot.KD);

    private static boolean hasInstance = false;

    private Shooter(ShooterPivotIO pivotIO, FlywheelsIO flywheelsIO) {
        if (hasInstance) throw new IllegalStateException("Instance of shooter already exists");
        hasInstance = true;
        this.pivotIO = pivotIO;
        this.flywheelsIO = flywheelsIO;
    }

    public static Shooter createReal() {
        if (Constants.currentMode != Constants.Mode.REAL) {
            DriverStation.reportWarning("Using real shooter on simulated robot", false);
        }
        return new Shooter(new ShooterPivotIOTalonFX(), new FlywheelsIOTalonFX());
//        return new Shooter(new ShooterPivotIO() {}, new FlywheelsIOTalonFX());
    }

    public static Shooter createSim() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using simulated shooter on real robot", false);
        }
        return new Shooter(new ShooterPivotIOSim(), new FlywheelsIOSim());
    }

    public static Shooter createDummy() {
        if (Constants.currentMode == Constants.Mode.REAL) {
            DriverStation.reportWarning("Using dummy shooter on real robot", false);
        }
        return new Shooter(new ShooterPivotIO(){}, new FlywheelsIO(){});
    }

    @Override
    public void periodic() {
        // Update & process inputs
        pivotIO.updateInputs(pivotInputs);
        flywheelsIO.updateInputs(flywheelInputs);
        Logger.processInputs("Shooter/Pivot", pivotInputs);
        Logger.processInputs("Shooter/Flywheels", flywheelInputs);
        MechanismVisualiser.setShooterPivotRotation(getPivotPosition());

        if (RobotState.isDisabled()){
            stopFlywheels();
            stopPivot();
        }

        // Update tunable numbers
        if (flywheelsKP.hasChanged(hashCode()) || flywheelsKI.hasChanged(hashCode()) || flywheelsKD.hasChanged(hashCode())) {
            flywheelsIO.configPID(flywheelsKP.get(), flywheelsKI.get(), flywheelsKD.get());
        }
        if (pivotKP.hasChanged(hashCode()) || pivotKI.hasChanged(hashCode()) || pivotKD.hasChanged(hashCode())) {
            pivotIO.configPID(pivotKP.get(), pivotKI.get(), pivotKD.get());
        }

        // Add values to filters
        leftSpeedFilter.add(getLeftFlywheelSpeed());
        rightSpeedFilter.add(getRightFlywheelSpeed());
        pivotPositionFilter.add(getPivotPosition().getRotations());
        Logger.recordOutput("Shooter/Pivot/Setpoint", pivotSetpoint);
        Logger.recordOutput("Shooter/Pivot/Error", pivotSetpoint.getDegrees() - pivotInputs.position.getDegrees());
    }

    /**
     * Sets the left and right speeds of the flywheels
     */
    public void setFlywheelSpeeds(double leftSpeedRPM, double rightSpeedRPM) {
        leftFlywheelSetpointRPM = leftSpeedRPM;
        rightFlywheelSetpointRPM = rightSpeedRPM;
        leftSpeedFilter.clear();
        rightSpeedFilter.clear();
        flywheelsIO.setSpeeds(leftSpeedRPM, rightSpeedRPM);
    }

    /**
     * Sets the voltages to each side of the flywheel
     */
    public void setFlywheelVolts(double rightVolts, double leftVolts) {
        flywheelsIO.setVoltage(
                MathUtil.clamp(rightVolts, -12, 12),
                MathUtil.clamp(leftVolts, -12, 12)
        );
    }

    /**
     * Applies neutral output to the flywheels
     */
    public void stopFlywheels() {
        setFlywheelVolts(0, 0);
    }

    /**
     * Sets the position of the pivot, using parallel to the floor as 0
     */
    public void setPivotPosition(Rotation2d position) {
        pivotSetpoint = Rotation2d.fromRotations(
                MathUtil.clamp(
                        position.getRotations(),
                        Pivot.MIN_ANGLE.getRotations(),
                        Pivot.MAX_ANGLE.getRotations()
                )
        );
        pivotPositionFilter.clear();
        pivotIO.setPosition(pivotSetpoint);
    }
    /**
     * Sets the position of the pivot, using parallel to the floor as 0
     */
    public void holdPivotPosition() {
//        pivotPositionFilter.clear();
        pivotIO.setPosition(pivotSetpoint);
    }

    /**
     * Sets the voltage to the pivot
     */
    public void setPivotVolts(double volts) {
        pivotIO.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    /**
     * Applies neutral output to the pivot
     */
    public void stopPivot() {
        setPivotVolts(0);
    }

    public void setPivotBrakeMode(boolean isBrakeMode) {
        pivotIO.setBrakeMode(isBrakeMode);
    }

    /**
     * Gets the speed of the left flywheel in RPM
     */
    public double getLeftFlywheelSpeed() {
        return flywheelInputs.leftVelocityRPM;
    }

    /**
     * Gets the speed of the right flywheel in RPM
     */
    public double getRightFlywheelSpeed() {
        return flywheelInputs.rightVelocityRPM;
    }

    /**
     * Gets the position of the pivot
     */
    public Rotation2d getPivotPosition() {
        return pivotInputs.position;
    }

    /**
     * Gets whether the flywheels are spun up to their setpoints
     */
    public boolean areFlywheelsSpunUp() {
        return
                MathUtil.isNear(leftFlywheelSetpointRPM, leftSpeedFilter.getAverage(), Flywheels.ERROR_TOLERANCE_RPM) &&
                        MathUtil.isNear(rightFlywheelSetpointRPM, rightSpeedFilter.getAverage(), Flywheels.ERROR_TOLERANCE_RPM);
    }

    /**
     * Gets whether the pivot is at its setpoint
     */
    public boolean isPivotAtSetpoint() {
        return MathUtil.isNear(
                pivotSetpoint.getRotations(), pivotPositionFilter.getAverage(), Pivot.ERROR_TOLERANCE.getRotations());
    }

    public Command spinUpCommand(Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint) {
        return new FunctionalCommand(
                () -> {},
                () -> setFlywheelSpeeds(leftSetpoint.get(), rightSetpoint.get()),
                (ignored) -> {},
                this::areFlywheelsSpunUp,
                this);
    }

    public Command setPivotPositionCommand(Supplier<Rotation2d> setpoint) {
        return new FunctionalCommand(
                () -> {},
                () -> setPivotPosition(setpoint.get()),
                (ignored) -> {},
                this::isPivotAtSetpoint,
                this);
    }

    public Command spinUpAndSetPivotPosition(Supplier<Double> leftSetpoint, Supplier<Double> rightSetpoint, Supplier<Rotation2d> setpoint){
        return new FunctionalCommand(
                () -> {},
                () -> {
                    setPivotPosition(setpoint.get());
                    setFlywheelSpeeds(leftSetpoint.get(), rightSetpoint.get());
                },
                (ignored) -> {},
                () -> areFlywheelsSpunUp() && isPivotAtSetpoint(),
                this
        );
    }

    @AutoLogOutput
    public double getLeftFlywheelSetpointRPM() {
        return leftFlywheelSetpointRPM;
    }

    @AutoLogOutput
    public double getRightFlywheelSetpointRPM() {
        return rightFlywheelSetpointRPM;
    }

    public void zeroPivot() {
        pivotIO.setEncoderPosition(0);
    }
}
