// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;

public class RobotContainer {
	Swerve swerve = Swerve.getInstance();

	@Getter
	private UpdateManager updateManager;

	CommandXboxController driverController = new CommandXboxController(0);

	public RobotContainer() {
		updateManager = new UpdateManager(
				swerve);
		updateManager.registerAll();

		configureBindings();
		System.out.println("Init Completed!");
	}

	/** Bind controller keys to commands */
	private void configureBindings() {
		swerve.setDefaultCommand(Commands
				.runOnce(() -> swerve.drive(
						new Translation2d(
								driverController.getLeftY(),
								driverController.getLeftX()),
						driverController.getRightX(),
						true,
						false),
						swerve));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
