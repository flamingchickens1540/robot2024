package org.team1540.robot2024.util.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.List;

public class AutoManager {

    private static AutoManager instance;
    private final LoggedDashboardChooser<AutoCommand> chooser = new LoggedDashboardChooser<>("Auto Choices");
    private final Field2d field2d = new Field2d();

    private AutoManager() {
        SmartDashboard.putData("field", field2d);
    }
    public static AutoManager getInstance() {
        if (instance == null) {

            instance = new AutoManager();
        }
        return instance;
    }

    public void add(AutoCommand command) {
        chooser.addOption(command.getName(), command);
    }
    public void addDefault(AutoCommand command) {
        chooser.addDefaultOption(command.getName(), command);
    }

    public void updateSelected() {
        AutoCommand selected = chooser.get();
        if (selected != null) {
            List<Pose2d> trajectory = selected.toTrajectory();
            if (trajectory != null) {
                field2d.getObject("trajectory").setPoses(trajectory);
            } else {
                field2d.getObject("trajectory").setPoses();
            }
        }
    }

    public Command getSelected() {
        return chooser.get();
    }
    public String getSelectedName() {
        return chooser.get().getName();
    }

    public boolean getSelectedShouldReset() {
        return chooser.get().getIsResetting();
    }
    public Pose2d getSelectedInitialPose() {
        return chooser.get().getInitialPose();
    }
}
