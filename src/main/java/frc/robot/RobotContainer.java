// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.SpeakerAimingCommand;
import frc.robot.commands.SpeakerShootCommand;
import frc.robot.subsystems.beambreak.BeamBreakIORev;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorIOARGB;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intaker.IntakerIOTalonFX;
import frc.robot.subsystems.intaker.IntakerSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.display.Display;
import lombok.Getter;

public class RobotContainer {
	IntakerSubsystem intakerSubsystem = new IntakerSubsystem(new IntakerIOTalonFX());
	IndexerSubsystem indexerSubsystem = new IndexerSubsystem(new IndexerIOTalonFX());
	ShooterSubsystem shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
	BeamBreakSubsystem beamBreakSubsystem = new BeamBreakSubsystem(new BeamBreakIORev());
	Limelight limelight = Limelight.getInstance();
	IndicatorSubsystem indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOARGB());

	Swerve swerve = Swerve.getInstance();
	Display display = Display.getInstance();
	

	CommandXboxController driverController = new CommandXboxController(0);
	CommandXboxController operatorController = new CommandXboxController(1);

	@Getter
	private UpdateManager updateManager;

	public RobotContainer() {
		updateManager = new UpdateManager(
			swerve,
			limelight,
			display
		);
		updateManager.registerAll();

		configureBindings();
		System.out.println("Init Completed!");
	}
	/**
	 * Bind controller keys to commands
	 */
	private void configureBindings() {
		// Drive mode 1
		swerve.setDefaultCommand(Commands
				.runOnce(() -> swerve.drive(
						new Translation2d(
								-driverController.getLeftY() * Constants.SwerveDrivetrain.maxSpeed.magnitude(),
								-driverController.getLeftX() * Constants.SwerveDrivetrain.maxSpeed.magnitude()),
						-Constants.RobotConstants.driverController.getRightX()
								* Constants.SwerveDrivetrain.maxAngularRate.magnitude(),
						true,
						false),
						swerve));
		// Drive mode 2
		// swerve.setDefaultCommand(Commands
		// .runOnce(() -> swerve.drive(
		// new Translation2d(
		// -
		// driverController.getLeftY()*Constants.SwerveDrivetrian.maxSpeed.magnitude(),
		// -
		// driverController.getRightX()*Constants.SwerveDrivetrian.maxSpeed.magnitude()),
		// (-Constants.RobotConstants.driverController.getRightTriggerAxis()
		// + Constants.RobotConstants.driverController.getLeftTriggerAxis())
		// * Constants.SwerveDrivetrian.maxAngularRate.magnitude(),
		// true,
		// false),
		// swerve));
		// Point Wheel
		// swerve.setDefaultCommand(Commands.runOnce(() -> swerve.pointWheelsAt(
		// new edu.wpi.first.math.geometry.Rotation2d(
		// driverController.getLeftX()*Math.PI/2)),
		// swerve));
		// field relative heading
		// driverController.a().

		// driverController.rightTrigger().whileTrue(
		// 		Commands.sequence(
		// 				new SpeakerShootCommand(
		// 						shooterSubsystem,
		// 						indexerSubsystem,
		// 						beamBreakSubsystem,
		// 						indicatorSubsystem,
		// 						swerve,
		// 						driverController,
		// 						() -> driverController.getHID().getRightBumper()),
		// 				new RumbleCommand(Seconds.of(1), driverController.getHID(),
		// 						operatorController.getHID())));
		driverController.rightBumper()
				.whileTrue(new SpeakerAimingCommand(shooterSubsystem, indicatorSubsystem, swerve, driverController));

		// driverController.rightTrigger().whileTrue(Commands.run(() -> {
		// 	swerve.setHeadingTarget(Limelight.getInstance().getSpeakerRelativePosition());
		// 	System.out.println(Limelight.getInstance().getSpeakerRelativePosition());
		// }, swerve));

		// driverController.a().onTrue(Commands.runOnce(()->swerve.setLockHeading(true), swerve));
		// driverController.b().onTrue(Commands.runOnce(() -> swerve.setLockHeading(false), swerve));
		// driverController.povLeft().onTrue(Commands.runOnce(() -> {
		// 	swerve.setLockHeading(true);
		// 	swerve.setHeadingTarget(90);
		// 	//swerve.setLockHeading(false);
		// 	System.out.println("aaaaa");}, swerve));

		driverController.leftBumper().whileTrue(
				Commands.sequence(
						Commands.parallel(
								new IntakeCommand(intakerSubsystem, beamBreakSubsystem,
										indicatorSubsystem, shooterSubsystem),
								new IndexCommand(indexerSubsystem, beamBreakSubsystem)),
						new RumbleCommand(Seconds.of(1), driverController.getHID(),
								operatorController.getHID())));
		driverController.start().onTrue(Commands.runOnce(() -> {
			swerve.resetHeadingController();
			// ControllerDriveCommand.facingAngle = 0.0;
			// Pigeon2 mPigeon2 = new Pigeon2(Constants.SwerveDrivetrian.PIGEON_ID,
			// Constants.RobotConstants.RobotConstants.CAN_BUS_NAME);
			edu.wpi.first.math.geometry.Rotation2d a = swerve.getLocalizer().getLatestPose().getRotation();// new
																											// edu.wpi.first.math.geometry.Rotation2d(mPigeon2.getYaw().getValueAsDouble());
			// swerve.getGyro().getYaw().;//.getLocalizer().getLatestPose().getRotation();
			System.out.println("A = " + a);
			Pose2d b = new Pose2d(new Translation2d(0, 0), a);
			swerve.resetPose(b);
		}));
		driverController.povUp().whileTrue(Commands.runOnce(()->System.out.println(swerve.getLocalizer().getLatestPose()),swerve));

	}

	public Command getAutonomousCommand() {
		// return new CharacterizationDriveCommand(swerve, 3, 1.5, 6);
		return Commands.print("No autonomous command configured");
	}
}
