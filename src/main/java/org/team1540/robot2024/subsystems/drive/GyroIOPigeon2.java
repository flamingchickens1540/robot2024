package org.team1540.robot2024.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.team1540.robot2024.util.PhoenixTimeSyncSignalRefresher;

import static org.team1540.robot2024.Constants.SwerveConfig.*;

/**
 * IO implementation for Pigeon2
 */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(PIGEON_ID, CAN_BUS);
    private final StatusSignal<Double> yaw = pigeon.getYaw();
    private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZDevice();

    private final PhoenixTimeSyncSignalRefresher odometrySignalRefresher;

    public GyroIOPigeon2(PhoenixTimeSyncSignalRefresher odometrySignalRefresher) {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(CAN_UPDATE_FREQUENCY_HZ);
        yawVelocity.setUpdateFrequency(CAN_UPDATE_FREQUENCY_HZ);
        pigeon.optimizeBusUtilization();

        this.odometrySignalRefresher = odometrySignalRefresher;
        odometrySignalRefresher.registerSignals(yaw, yawVelocity);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        odometrySignalRefresher.refreshSignals();
        inputs.connected = BaseStatusSignal.isAllGood(yaw, yawVelocity);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }
}
