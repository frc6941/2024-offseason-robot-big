// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.display.Display;
import frc.robot.subsystems.beambreak.BeamBreakIORev;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorIOARGB;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intaker.IntakerIOTalonFX;
import frc.robot.subsystems.intaker.IntakerSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import org.frcteam6941.looper.UpdateManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static edu.wpi.first.units.Units.Seconds;

public class RobotContainer {
    private final LoggedDashboardChooser<Command> autoChooser;
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

        autoChooser = new LoggedDashboardChooser<>("Chooser", AutoBuilder.buildAutoChooser());
        autoChooser.addOption(
                "Flywheel SysId (Quasistatic Forward)",
                shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Flywheel SysId (Quasistatic Reverse)",
                shooterSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Flywheel SysId (Dynamic Forward)", shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Flywheel SysId (Dynamic Reverse)", shooterSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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

        driverController.rightTrigger().whileTrue(
                Commands.sequence(
                        new SpeakerShootCommand(
                                shooterSubsystem,
                                indexerSubsystem,
                                beamBreakSubsystem,
                                indicatorSubsystem,
                                swerve,
                                driverController,
                                () -> driverController.getHID().getRightBumper()),
                        new RumbleCommand(Seconds.of(1), driverController.getHID(),
                                operatorController.getHID())));
        //driverController.rightTrigger().onFalse(new ResetArmCommand(shooterSubsystem));

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
            edu.wpi.first.math.geometry.Rotation2d a = swerve.getLocalizer().getLatestPose().getRotation();
            System.out.println("A = " + a);
            Pose2d b = new Pose2d(new Translation2d(0, 0), a);
            swerve.resetPose(b);
        }));
        driverController.povUp().whileTrue(new SetFacingCommand(swerve, 0));
        driverController.povUpRight().whileTrue(new SetFacingCommand(swerve, 315));
        driverController.povRight().whileTrue(new SetFacingCommand(swerve, 270));
        driverController.povDownRight().whileTrue(new SetFacingCommand(swerve, 225));
        driverController.povDown().whileTrue(new SetFacingCommand(swerve, 180));
        driverController.povDownLeft().whileTrue(new SetFacingCommand(swerve, 135));
        driverController.povLeft().whileTrue(new SetFacingCommand(swerve, 90));
        driverController.povUpLeft().whileTrue(new SetFacingCommand(swerve, 45));
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
			edu.wpi.first.math.geometry.Rotation2d a = swerve.getLocalizer().getLatestPose().getRotation();
			System.out.println("A = " + a);
			Pose2d b = new Pose2d(new Translation2d(3,7.841), a);
			swerve.resetPose(b);
		}));
		driverController.povUp().whileTrue(new SetFacingCommand(swerve, 0));
		driverController.povUpRight().whileTrue(new SetFacingCommand(swerve, 315));
		driverController.povRight().whileTrue(new SetFacingCommand(swerve,270));
		driverController.povDownRight().whileTrue(new SetFacingCommand(swerve, 225));
		driverController.povDown().whileTrue(new SetFacingCommand(swerve, 180));
		driverController.povDownLeft().whileTrue(new SetFacingCommand(swerve, 135));
		driverController.povLeft().whileTrue(new SetFacingCommand(swerve, 90));
		driverController.povUpLeft().whileTrue(new SetFacingCommand(swerve, 45));

        // driverController.x()
        // 		.onTrue(Commands.runOnce(() -> indicatorSubsystem.setPattern(IndicatorIO.Patterns.NORMAL),
        // 				indicatorSubsystem));
        indicatorSubsystem.setDefaultCommand(Commands.run(() -> indicatorSubsystem.setPattern(IndicatorIO.Patterns.NORMAL), indicatorSubsystem));

        driverController.y().onTrue(new ResetArmCommand(shooterSubsystem));

    }

    public Command getAutonomousCommand() {
        // return new CharacterizationDriveCommand(swerve, 3, 1.5, 6);
        return autoChooser.get();
    }
}
