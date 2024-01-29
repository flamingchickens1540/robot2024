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
    private final Pigeon2 pigeon = new Pigeon2(PIGEON_ID);
    private final StatusSignal<Double> yaw = pigeon.getYaw();
    private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZDevice();

    private final PhoenixTimeSyncSignalRefresher timeSyncSignalRefresher;

    public GyroIOPigeon2(PhoenixTimeSyncSignalRefresher timeSyncSignalRefresher) {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(100.0);
        yawVelocity.setUpdateFrequency(100.0);
        pigeon.optimizeBusUtilization();

        this.timeSyncSignalRefresher = timeSyncSignalRefresher;
        timeSyncSignalRefresher.registerSignals(yaw, yawVelocity);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        timeSyncSignalRefresher.refreshSignals();
        inputs.connected = BaseStatusSignal.isAllGood(yaw, yawVelocity);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }
}
