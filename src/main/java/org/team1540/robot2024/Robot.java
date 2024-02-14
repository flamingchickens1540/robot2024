package org.team1540.robot2024;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team1540.robot2024.subsystems.led.*;
import org.team1540.robot2024.subsystems.led.patterns.LedPatternFlame;
import org.team1540.robot2024.subsystems.led.patterns.LedPatternRSLState;
import org.team1540.robot2024.subsystems.led.patterns.LedPatternRainbow;
import org.team1540.robot2024.subsystems.led.patterns.SimpleLedPattern;
import org.team1540.robot2024.util.LoggedTunableNumber;
import org.team1540.robot2024.util.MechanismVisualiser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    LoggedTunableNumber led_r = new LoggedTunableNumber("led/r", 0);
    LoggedTunableNumber led_g = new LoggedTunableNumber("led/g", 0);
    LoggedTunableNumber led_b = new LoggedTunableNumber("led/b", 0);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
//                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.disableDeterministicTimestamps()

        // Start AdvantageKit logger
        Logger.start();

        DriverStation.silenceJoystickConnectionWarning(true);
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    /**
     * This function is called periodically during all modes.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();
        if (Constants.currentMode == Constants.Mode.REAL) robotContainer.odometrySignalRefresher.periodic();

        // Update mechanism visualiser in sim
        if (Robot.isSimulation()) MechanismVisualiser.periodic();
//        robotContainer.leds.setPattern(Leds.Zone.ZONE1, SimpleLedPattern.alternating(Color.kBlueViolet, Color.kGreen));
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        robotContainer.leds.setPattern(Leds.Zone.ELEVATOR_BACK, new LedPatternFlame());
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        robotContainer.leds.setPattern(Leds.Zone.ELEVATOR_BACK,LedPatternRSLState.matchingColors());
        autonomousCommand = robotContainer.getAutonomousCommand();
        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.

        robotContainer.leds.setPattern(Leds.Zone.ELEVATOR_BACK, LedPatternRSLState.matchingColors());
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        robotContainer.leds.setPattern(Leds.Zone.ELEVATOR_BACK,new LedPatternRainbow(1));
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        robotContainer.leds.setPattern(Leds.Zone.ELEVATOR_BACK,SimpleLedPattern.solid(new Color(led_r.get(), led_g.get(), led_b.get())));
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
