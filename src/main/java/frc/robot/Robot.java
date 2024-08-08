// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;


public class Robot extends LoggedRobot {
    CommandXboxController driverController = new CommandXboxController(0);
    private Command m_autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        DriverStation.silenceJoystickConnectionWarning(true);
        // robotContainer.getUpdateManager().startEnableLoop(Constants.LOOPER_DT);
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
        robotContainer.getUpdateManager().runEnableSingle();
        m_autonomousCommand = robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        robotContainer.getUpdateManager().invokeStart();
    }

    @Override
    public void autonomousPeriodic() {
        robotContainer.getUpdateManager().runEnableSingle();
    }

    @Override
    public void autonomousExit() {
        robotContainer.getUpdateManager().invokeStop();
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
