// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ResetArmCommand;
import frc.robot.display.OperatorDashboard;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.limelight.Light;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;


public class Robot extends LoggedRobot {
    ShooterSubsystem shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
    ArmSubsystem armSubsystem = new ArmSubsystem(new ArmIOTalonFX());
    OperatorDashboard dashboard = OperatorDashboard.getInstance();
    private Command m_autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        Light.getInstance().setState(Light.STATE.OFF);
        robotContainer = new RobotContainer();
        DriverStation.silenceJoystickConnectionWarning(true);
        robotContainer.getUpdateManager().startEnableLoop(Constants.LOOPER_DT);
        FollowPathCommand.warmupCommand().schedule();
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        new ResetArmCommand(armSubsystem).schedule();
    }

    @Override
    public void robotPeriodic() {
        // timer.UpdateTimer();
        CommandScheduler.getInstance().run();
        robotContainer.getUpdateManager().runEnableSingle();
    }

    @Override
    public void disabledInit() {
        Light.getInstance().setState(Light.STATE.OFF);
    }

    @Override
    public void disabledPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        Light.getInstance().setState(Light.STATE.ON);
        robotContainer.getUpdateManager().runEnableSingle();
        m_autonomousCommand = robotContainer.getAutonomousCommand();
//        robotContainer.resetOdomAuto().schedule();
//        new ResetArmCommand(armSubsystem).schedule();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        robotContainer.getUpdateManager().invokeStart();
        Swerve.getInstance().auto();
        Commands.runOnce(() -> dashboard.updateDestination(ShootingDecider.Destination.SPEAKER));
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousPeriodic() {
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
        Light.getInstance().setState(Light.STATE.AUTO);
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

    @Override
    public void simulationInit() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    }

    @Override
    public void simulationPeriodic() {
        robotContainer.getUpdateManager().runSimulateSingle();
    }
}
