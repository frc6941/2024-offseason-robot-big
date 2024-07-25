// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.a.TunableNumber;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;

	private RobotContainer m_robotContainer;

	@Override
	public void robotInit() {
		m_robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
		m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}
	
	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
		m_autonomousCommand.cancel();
		}
	}

	// SwerveDrivetrain m_drivetrain = new SwerveDrivetrain(Constants.SwerveDrivetrian.DrivetrainConstants,
	// 		Constants.SwerveDrivetrian.modules);
	@Override
	public void teleopPeriodic() {
		// m_drivetrain.setControl(
		// 		SwerveSubsystem.m_driveRequest
		// 				.withVelocityX(
		// 						Utils.sign(-Constants.RobotConstants.driverController.getLeftY())
		// 						* Constants.SwerveDrivetrian.xLimiter.calculate(Math.abs(Constants.RobotConstants.driverController.getLeftY()))
		// 						* Constants.SwerveDrivetrian.maxSpeed.magnitude()) 
		// 				.withVelocityY(
		// 						Utils.sign(-Constants.RobotConstants.driverController.getLeftX())
		// 						* Constants.SwerveDrivetrian.yLimiter.calculate(Math.abs(Constants.RobotConstants.driverController.getLeftX()))
		// 						* Constants.SwerveDrivetrian.maxSpeed.magnitude()) 
		// 				.withRotationalRate(
		// 						-Constants.RobotConstants.driverController.getRightX()
		// 						* Constants.SwerveDrivetrian.maxAngularRate.magnitude()));
		}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
