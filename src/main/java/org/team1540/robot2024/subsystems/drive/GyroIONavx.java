package org.team1540.robot2024.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class GyroIONavx implements GyroIO{
    private final AHRS navx = new AHRS(SPI.Port.kMXP);
    private Rotation2d lastAngle;

    public GyroIONavx(){
        lastAngle = navx.getRotation2d();
    }


    @Override
    public void updateInputs(GyroIOInputs inputs) {
        double lastTime = inputs.time;
        Rotation2d angle = navx.getRotation2d();
        inputs.time = Timer.getFPGATimestamp();
        inputs.connected = true;
        inputs.yawPosition = angle;
        inputs.yawVelocityRadPerSec = (angle.minus(lastAngle).getRadians())/(inputs.time - lastTime);
        lastAngle = angle;
    }
}
