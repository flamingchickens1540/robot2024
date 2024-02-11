package org.team1540.robot2024.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoManager {

    private static AutoManager instance;
    private final LoggedDashboardChooser<AutoCommand> chooser = new LoggedDashboardChooser<>("Auto Choices");

    public static AutoManager getInstance() {
        if (instance == null) {
            instance = new AutoManager();
        }
        return instance;
    }

    public void addAuto(AutoCommand command) {
        chooser.addOption(command.getName(), command);
    }
    public void addDefaultAuto(AutoCommand command) {
        chooser.addDefaultOption(command.getName(), command);
    }

    public void updateSelected() {
        AutoCommand selected = chooser.get();
//        if (selected != null) {
//            if (selected.trajectory != null) {
//                field2d.getObject("trajectory").setTrajectory(TrajectoryTransformer.transformTrajectoryForAlliance(selected.trajectory, DriverStation.getAlliance()));
//            } else {
//                field2d.getObject("trajectory").setPoses();
//            }
//        }
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
