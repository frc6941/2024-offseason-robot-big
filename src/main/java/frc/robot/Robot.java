// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.display.OperatorDashboard;
import frc.robot.subsystems.limelight.Light;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;


public class Robot extends LoggedRobot {
    OperatorDashboard dashboard = OperatorDashboard.getInstance();
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.start();

        DriverStation.silenceJoystickConnectionWarning(true);

        robotContainer = new RobotContainer();
        robotContainer.getUpdateManager().startEnableLoop(Constants.LOOPER_DT);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.getUpdateManager().runEnableSingle();
    }

    @Override
    public void disabledInit() {
        Light.getInstance().setState(Light.STATE.OFF);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        Light.getInstance().setState(Light.STATE.ON);
        robotContainer.getUpdateManager().runEnableSingle();
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
        robotContainer.getUpdateManager().invokeStart();
        Swerve.getInstance().auto();
        Commands.runOnce(() -> dashboard.updateDestination(ShootingDecider.Destination.SPEAKER));
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
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
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
