package org.team1540.robot2024.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class GyroIONavx implements GyroIO {
    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private Rotation2d lastAngle;
    private double lastTime;

    public GyroIONavx() {
        lastAngle = navx.getRotation2d();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        Rotation2d angle = navx.getRotation2d();
        inputs.connected = navx.isConnected();
        inputs.yawPosition = angle;

        double time = Timer.getFPGATimestamp();
        inputs.yawVelocityRadPerSec = (angle.minus(lastAngle).getRadians()) / (time - lastTime);
        lastTime = time;
        lastAngle = angle;
    }
}
