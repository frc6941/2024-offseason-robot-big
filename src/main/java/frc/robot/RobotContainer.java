// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.swerve.ControllerDriveCommand;
import frc.robot.commands.swerve.SetFieldCentricCommand;
import frc.robot.commands.swerve.SwerveDrivetrainSpinCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
//import frc.robot.Constants;

public class RobotContainer {
	//Define the subsystems and configure them in detail in configureSubsystems()
	SwerveSubsystem swerveSubsystem;

	public RobotContainer() {
		configureSubsystems();
		System.out.println("Subsystems Init Completed!");
		configureBindings();
		System.out.println("Init Completed!");
	}

	/** Every subsystem should be defined here */
	private void configureSubsystems() {
		swerveSubsystem = new SwerveSubsystem(
				Constants.SwerveDrivetrian.DrivetrainConstants, 
				Constants.SwerveDrivetrian.modules);	
	}

	/** Bind controller keys to commands */
	private void configureBindings() 
	{
		//SuerveSubsystem
		//Driving follow the controller in field centric mode
		//Drive mode 1
		// swerveSubsystem.setDefaultCommand(
		// 	new ControllerDriveCommand(
		// 			swerveSubsystem,
		// 			() -> Constants.RobotConstants.driverController.getLeftY(),
		// 			() -> Constants.RobotConstants.driverController.getLeftX(),
		// 			() -> Constants.RobotConstants.driverController.getRightX()));
		//Drive mode 2			
		swerveSubsystem.setDefaultCommand(
			new ControllerDriveCommand(
					swerveSubsystem,
					() -> Constants.RobotConstants.driverController.getLeftY(),
					() -> Constants.RobotConstants.driverController.getRightX(),
					() -> Constants.RobotConstants.driverController.getRightTriggerAxis()
					- Constants.RobotConstants.driverController.getLeftTriggerAxis()));
		//Press start to change the center of the field
		Constants.RobotConstants.driverController.start()
				.onTrue(new SetFieldCentricCommand(swerveSubsystem));
		//Press A to trun 45 degrees in clockwise
		Constants.RobotConstants.driverController.a()
				.onTrue(new SwerveDrivetrainSpinCommand(swerveSubsystem, 45.0));

	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
