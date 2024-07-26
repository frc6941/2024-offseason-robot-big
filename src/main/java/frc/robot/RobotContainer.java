// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frcteam6941.looper.UpdateManager;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Pose2d;
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
		updateManager = new UpdateManager(swerve);
		updateManager.registerAll();

		configureBindings();
		System.out.println("Init Completed!");
	}

	/** Bind controller keys to commands */
	private void configureBindings() {
		swerve.setDefaultCommand(Commands
				.runOnce(() -> swerve.drive(
						new Translation2d(
								driverController.getLeftY()*Constants.SwerveDrivetrian.maxSpeed.magnitude(),
								driverController.getRightX()*Constants.SwerveDrivetrian.maxSpeed.magnitude()),
						Constants.RobotConstants.driverController.getRightTriggerAxis()
						- Constants.RobotConstants.driverController.getLeftTriggerAxis(),
						true,
						true),
						swerve));
		driverController.start().onTrue(Commands.runOnce(() -> {
				Translation2d a = swerve.getLocalizer().getLatestPose().getTranslation();
				Pose2d b = new Pose2d(a, new edu.wpi.first.math.geometry.Rotation2d());
				swerve.resetPose(b);}));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
