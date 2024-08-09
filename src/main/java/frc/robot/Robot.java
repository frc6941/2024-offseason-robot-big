// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;


public class Robot extends LoggedRobot {
    CommandXboxController driverController = new CommandXboxController(0);
    Swerve swerve;
    private Command m_autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        DriverStation.silenceJoystickConnectionWarning(true);
        // robotContainer.getUpdateManager().startEnableLoop(Constants.LOOPER_DT);
        FollowPathCommand.warmupCommand().schedule();
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        // timer.UpdateTimer();
        CommandScheduler.getInstance().run();
        robotContainer.getUpdateManager().runEnableSingle();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        robotContainer.getUpdateManager().runEnableSingle();
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        Commands.runOnce(() -> {
            swerve.resetHeadingController();
            swerve.resetPose(new Pose2d(new Translation2d(1.401, 5.551), swerve.getLocalizer().getLatestPose().getRotation()));
        });
        robotContainer.getUpdateManager().runEnableSingle();
        m_autonomousCommand = robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        robotContainer.getUpdateManager().invokeStart();
        Swerve.getInstance().auto();
    }

    @Override
    public void autonomousPeriodic() {
        robotContainer.getUpdateManager().runEnableSingle();
    }

    @Override
    public void autonomousExit() {
        robotContainer.getUpdateManager().invokeStop();
        Swerve.getInstance().normal();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        robotContainer.getUpdateManager().invokeStart();

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        robotContainer.getUpdateManager().invokeStop();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
