package org.team1540.robot2024.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.team1540.robot2024.util.LoggedTunableNumber;
import org.team1540.robot2024.util.math.AverageFilter;

import static org.team1540.robot2024.Constants.Shooter.*;

public class Shooter extends SubsystemBase {
    private final ShooterPivotIO pivotIO;
    private final ShooterPivotIOInputsAutoLogged pivotInputs = new ShooterPivotIOInputsAutoLogged();

    private final FlywheelsIO flywheelsIO;
    private final FlywheelsIOInputsAutoLogged flywheelInputs = new FlywheelsIOInputsAutoLogged();

    private final AverageFilter leftSpeedFilter = new AverageFilter(20);
    private final AverageFilter rightSpeedFilter = new AverageFilter(20);
    private final AverageFilter pivotPositionFilter = new AverageFilter(10); // Units: rotations

    private double leftFlywheelSetpointRPM;
    private double rightFlywheelSetpointRPM;
    private Rotation2d pivotSetpoint;

    private final LoggedTunableNumber flywheelsKP = new LoggedTunableNumber("Shooter/Flywheels/kP");
    private final LoggedTunableNumber flywheelsKI = new LoggedTunableNumber("Shooter/Flywheels/kI");
    private final LoggedTunableNumber flywheelsKD = new LoggedTunableNumber("Shooter/Flywheels/kD");

    private final LoggedTunableNumber pivotKP = new LoggedTunableNumber("Shooter/Pivot/kP");
    private final LoggedTunableNumber pivotKI = new LoggedTunableNumber("Shooter/Pivot/kI");
    private final LoggedTunableNumber pivotKD = new LoggedTunableNumber("Shooter/Pivot/kD");

    public Shooter(ShooterPivotIO pivotIO, FlywheelsIO flywheelsIO) {
        this.pivotIO = pivotIO;
        this.flywheelsIO = flywheelsIO;

        // Init tunable numbers
        flywheelsKP.initDefault(Flywheels.KP);
        flywheelsKI.initDefault(Flywheels.KI);
        flywheelsKD.initDefault(Flywheels.KD);

        pivotKP.initDefault(Pivot.KP);
        pivotKI.initDefault(Pivot.KI);
        pivotKD.initDefault(Pivot.KD);
    }

    @Override
    public void periodic() {
        // Update & process inputs
        pivotIO.updateInputs(pivotInputs);
        flywheelsIO.updateInputs(flywheelInputs);
        Logger.processInputs("Shooter/Pivot", pivotInputs);
        Logger.processInputs("Shooter/Flywheels", flywheelInputs);

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
                MathUtil.isNear(leftFlywheelSetpointRPM, leftSpeedFilter.getAverage(), 50) &&
                MathUtil.isNear(rightFlywheelSetpointRPM, rightSpeedFilter.getAverage(), 50);
    }

    /**
     * Gets whether the pivot is at its setpoint
     */
    public boolean isPivotAtSetpoint() {
        return MathUtil.isNear(pivotSetpoint.getRotations(), pivotPositionFilter.getAverage(), 0.2/360);
    }
}
