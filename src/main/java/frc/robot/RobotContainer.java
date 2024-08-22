// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.auto.IndexAutoCommand;
import frc.robot.commands.auto.IntakeAutoCommand;
import frc.robot.commands.manual.ShootManualCommand;
import frc.robot.display.Display;
import frc.robot.display.OperatorDashboard;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.beambreak.BeamBreakIORev;
import frc.robot.subsystems.beambreak.BeamBreakIOSim;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorIOARGB;
import frc.robot.subsystems.indicator.IndicatorIOSim;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intaker.IntakerIOSim;
import frc.robot.subsystems.intaker.IntakerIOTalonFX;
import frc.robot.subsystems.intaker.IntakerSubsystem;
import frc.robot.subsystems.limelight.Light;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.shooting.ShootingDecider;
import frc.robot.utils.shooting.ShootingDecider.Destination;
import lombok.Getter;
import org.frcteam6941.looper.UpdateManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Seconds;

public class RobotContainer {
    private static Map<Destination, Command> shootingCommandMapping;
    @Getter
    private final UpdateManager updateManager;
    IntakerSubsystem intaker;
    IndexerSubsystem indexer;
    ShooterSubsystem shooter;
    ArmSubsystem arm;
    BeamBreakSubsystem beamBreak;
    IndicatorSubsystem indicator;
    Limelight limelight = Limelight.getInstance();
    Swerve swerve = Swerve.getInstance();
    ShootingDecider decider = ShootingDecider.getInstance();
    Display display = Display.getInstance();
    OperatorDashboard dashboard = OperatorDashboard.getInstance();
    CommandXboxController driverController = new CommandXboxController(0);
    CommandXboxController operatorController = new CommandXboxController(1);
    @Getter
    private LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        configureSubsystems();
        updateManager = new UpdateManager(
                swerve,
                limelight,
                display,
                decider);
        updateManager.registerAll();

        shootingCommandMapping = new HashMap<>();
        shootingCommandMapping.put(Destination.FERRY, ferryShot());
        shootingCommandMapping.put(Destination.AMP, ampShot());
        shootingCommandMapping.put(Destination.SPEAKER, speakerShot());

        configureAuto();
        configureBindings();
    }

    public void configureSubsystems() {
        if (RobotBase.isReal()) {
            intaker = new IntakerSubsystem(new IntakerIOTalonFX());
            indexer = new IndexerSubsystem(new IndexerIOTalonFX());
            shooter = new ShooterSubsystem(new ShooterIOTalonFX());
            beamBreak = new BeamBreakSubsystem(new BeamBreakIORev());
            indicator = new IndicatorSubsystem(new IndicatorIOARGB());
            arm = new ArmSubsystem(new ArmIOTalonFX());
        } else {
            intaker = new IntakerSubsystem(new IntakerIOSim());
            indexer = new IndexerSubsystem(new IndexerIOSim());
            shooter = new ShooterSubsystem(new ShooterIOSim());
            beamBreak = new BeamBreakSubsystem(new BeamBreakIOSim());
            indicator = new IndicatorSubsystem(new IndicatorIOSim());
            arm = new ArmSubsystem(new ArmIOSim());
        }
    }

    private void configureAuto() {
        NamedCommands.registerCommand("AutoShoot", speakerAutoShot().withTimeout(3.0));
        NamedCommands.registerCommand("Intake", intakeAuto().withTimeout(2.0));
        NamedCommands.registerCommand("IntakeOut", outtake().withTimeout(0.5));
        NamedCommands.registerCommand("ResetArm", new ResetArmCommand(arm));
        NamedCommands.registerCommand("FlyWheelRampUp", new FlyWheelRampUp(shooter, () -> Destination.SPEAKER));//READ ME change all "Preshoot" in auto files
        NamedCommands.registerCommand("AutoPreArm", new ArmAimCommand(arm, () -> Destination.SPEAKER));
        NamedCommands.registerCommand("ChassisAim", new ChassisAimCommand(swerve, () -> Destination.SPEAKER, () -> 0, () -> 0));
        NamedCommands.registerCommand("Shoot", new DeliverNoteCommand(indexer, beamBreak, indicator).withTimeout(1.0));
        NamedCommands.registerCommand("PreloadShoot", PreloadShoot().withTimeout(2.0));


        AutoBuilder.configureHolonomic(
                () -> Swerve.getInstance().getLocalizer().getCoarseFieldPose(0),
                (Pose2d pose2d) -> Swerve.getInstance().resetPose(pose2d),
                () -> Swerve.getInstance().getChassisSpeeds(),
                (ChassisSpeeds chassisSpeeds) -> Swerve.getInstance().driveSpeed(chassisSpeeds),
                new HolonomicPathFollowerConfig(
                        //    new PIDConstants(
                        //            Constants.AutoConstants.swerveXGainsClass.swerveX_KP.get(),
                        //            Constants.AutoConstants.swerveXGainsClass.swerveX_KI.get(),
                        //            Constants.AutoConstants.swerveXGainsClass.swerveX_KD.get()
                        //    ),
                        //    new PIDConstants(
                        //            Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KP.get(),
                        //            Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KI.get(),
                        //            Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KD.get()
                        //    ),
                        Constants.SwerveConstants.maxSpeed.magnitude(),
                        0.55,
                        new ReplanningConfig()),
                AllianceFlipUtil::shouldFlip,
                swerve
        );

        autoChooser = new LoggedDashboardChooser<>("Chooser", AutoBuilder.buildAutoChooser());

        // autoChooser.addOption(
        //         "Flywheel SysId (Quasistatic Forward)",
        //         shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Flywheel SysId (Quasistatic Reverse)",
        //         shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption(
        //         "Flywheel SysId (Dynamic Forward)", shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Flywheel SysId (Dynamic Reverse)", shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        dashboard.registerAutoSelector(autoChooser.getSendableChooser());
    }

    public void configureBindings() {
        /*
         * ------- Driver Keymap -------
         * Driving:
         * Left Joystick - Panning
         * Right Joystick - Spinning
         * D-Pad - Field-Oriented Facing
         * Start Button - Reset Odometry

         *
         * Superstructure:
         * X - Speaker Shot
         * Y - Amp Shot
         * A - Amp Confirmation (TODO: Consider remove)
         * B - Ferry Shot
         * LB - Intake
         * LT - Outtake
         *
         *
         * ------- Operator Keymap -------
         * Superstructure: (TODO: Consider remove)
         * D-Pad Up - Select Speaker
         * D-Pad Down - Select Amp
         * D-Pad right - Select Ferry
         *  Back Button - Reset Arm
         */

        swerve.setDefaultCommand(drive());
        indicator.setDefaultCommand(Commands.run(() -> {
            if (beamBreak.isIntakeReady())
                indicator.setPattern(IndicatorIO.Patterns.INDEXED);
            else
                indicator.setPattern(IndicatorIO.Patterns.NORMAL);
        }, indicator));

        driverController.start().onTrue(resetOdom());
        driverController.leftBumper().onTrue(
                intake().andThen(rumbleDriver(1.0)));

        // superstructure
        driverController.x().whileTrue(
                ferryShot()
                        .alongWith(setDest(Destination.FERRY))
                        .andThen(rumbleDriver(1.0)));

        driverController.b().whileTrue(

                speakerShot()
                        .alongWith(setDest(Destination.SPEAKER))
                        .andThen(rumbleDriver(1.0)));
        driverController.leftTrigger().whileTrue((justShoot().withTimeout(0.3)));
        driverController.rightTrigger().whileTrue(ampAim());

        driverController.y().whileTrue(outtake());

        // operator superstructure commands
        operatorController.povUp().onTrue(LightAuto());
        operatorController.povLeft().onTrue(LightOn());
        operatorController.povRight().onTrue(LightOff());

        operatorController.a().debounce(0.5).onTrue(climbUp());
        operatorController.b().debounce(0.2).onTrue(climbDown());

        operatorController.start().onTrue(new ResetArmHomeCommand(arm));

        operatorController.leftTrigger().whileTrue(ManualShoot());
        operatorController.rightTrigger().onTrue(justShoot());
    }


    public Command getAutonomousCommand() {
        // return Commands.parallel(resetOdomAuto(), new ResetArmCommand(arm), autoChooser.get());
        return autoChooser.get();
        // return AutoBuilder.buildAuto("S2-S-A1-A2-A3");
    }

    // command composer
    private Command rumbleDriver(double seconds) {
        return new RumbleCommand(Seconds.of(seconds), driverController.getHID());
    }

    private Command rumbleOperator(double seconds) {
        return new RumbleCommand(Seconds.of(seconds), operatorController.getHID());
    }

    private Command justShoot() {
        return new DeliverNoteCommand(indexer, beamBreak, indicator);
    }

    private Command ferryShot() {
        return new FerryShootCommand(shooter, arm, indexer, beamBreak, indicator, swerve,
                driverController::getLeftX, driverController::getLeftY);
    }

    private Command ampShot() {
        return new AmpShootCommand(shooter, arm, indexer, beamBreak, indicator, () -> driverController.a().getAsBoolean());
    }

    private Command ampAim() {
        return new ParallelCommandGroup(
                new ArmAimCommand(arm, () -> Destination.AMP),
                new FlyWheelRampUp(shooter, () -> Destination.AMP));
    }

    private Command speakerShot() {
        return new SpeakerShootCommand(shooter, arm, indexer, beamBreak, indicator, swerve,
                driverController::getLeftX, driverController::getLeftY, false);
    }

    private Command speakerAutoShot() {
        //return new SpeakerShootAutoCommand(shooter, indexer, beamBreak, indicator, swerve);
        return new SpeakerShootCommand(shooter, arm, indexer, beamBreak, indicator, swerve);
    }

    private Command preheat() {
        return new FlyWheelRampUp(shooter, () -> dashboard.getCurrDestination());
//                .alongWith(
//                new ArmAimCommand(arm, () -> dashboard.getCurrDestination())
//        ).alongWith(
//                Commands.print("Preheat Started!")
//        ).handleInterrupt(() -> {
//            System.out.println("Preheat Interrupted, Driver Take Control!");
//        });
    }

    private Command selectShot() {
        return Commands.select(shootingCommandMapping, dashboard::getCurrDestination);
    }

    private Command facing(double fieldAngleDeg) {
        return new SetFacingCommand(swerve, fieldAngleDeg);
    }

    private Command intakeAuto() {
        return new IntakeAutoCommand(intaker, beamBreak, shooter, arm)
                .alongWith(new IndexAutoCommand(indexer, beamBreak, indicator));
    }

    private Command intake() {
        return new IntakeCommand(intaker, beamBreak, shooter, arm)
                .alongWith(new IndexCommand(indexer, beamBreak, indicator));
    }

    private Command outtake() {
        return new IntakeOutCommand(intaker)
                .alongWith(new IndexOutCommand(indexer))
                .alongWith(new ArmOutCommand(arm));
    }

    private Command drive() {
        return Commands.runOnce(() -> {
            if (AllianceFlipUtil.shouldFlip()) {
                Translation2d transVel = new Translation2d(
                        -driverController.getLeftY(),
                        -driverController.getLeftX()).times(Constants.SwerveConstants.maxSpeed.magnitude()).rotateBy(Rotation2d.fromDegrees(180));
                double rotVel = -Constants.RobotConstants.driverController.getRightX()
                        * Constants.SwerveConstants.maxAngularRate.magnitude();
                swerve.drive(transVel, rotVel, true, false);
            } else {
                Translation2d transVel = new Translation2d(
                        -driverController.getLeftY(),
                        -driverController.getLeftX()).times(Constants.SwerveConstants.maxSpeed.magnitude());
                double rotVel = -Constants.RobotConstants.driverController.getRightX()
                        * Constants.SwerveConstants.maxAngularRate.magnitude();
                swerve.drive(transVel, rotVel, true, false);
            }
        }, swerve);
    }

    private Command resetOdom() {
        return Commands.runOnce(() -> {
            swerve.resetHeadingController();
            swerve.resetPose(
                    new Pose2d(AllianceFlipUtil.apply(Constants.FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()),
                            Rotation2d.fromDegrees(swerve.getLocalizer().getLatestPose().getRotation().getDegrees())));
            indicator.setPattern(IndicatorIO.Patterns.RESET_ODOM);
        }).ignoringDisable(true);
    }

    public Command resetOdomAuto() {
        return Commands.runOnce(() -> {
            swerve.resetHeadingController();
            swerve.resetPose(swerve.getLocalizer().getLatestPose());
        }).ignoringDisable(true);
    }

    private Command setDest(Destination des) {
        return Commands.runOnce(() -> dashboard.updateDestination(des));
    }

    private Command climbUp() {
        return new ClimbArmUpCommand(arm);
    }

    private Command climbDown() {
        return new ClimbPullerDownCommand(arm, indicator);
    }

    private Command PreloadShoot() {
        return Commands.deadline(
                Commands.sequence(
                        new WaitUntilCommand(() -> {
                            boolean shooterReady = shooter.ShooterVelocityReady();
                            boolean armReady = arm.armAimingReady();
                            return shooterReady && armReady;
                        }),
                        Commands.runOnce(() -> Timer.delay(0.02)),
                        new DeliverNoteCommand(indexer, beamBreak, indicator)),
                new ArmAimCommand(arm, () -> Destination.SPEAKER),
                new FlyWheelRampUp(shooter, () -> Destination.SPEAKER),
                Commands.runOnce(() -> indicator.setPattern(IndicatorIO.Patterns.SPEAKER_AIMING), indicator)
        );
    }

    private Command LightOn() {
        return Commands.run(() ->
                Light.getInstance().setState(Light.STATE.ON), Light.getInstance()).ignoringDisable(true);
    }

    private Command LightOff() {
        return Commands.run(() ->
                Light.getInstance().setState(Light.STATE.OFF), Light.getInstance()).ignoringDisable(true);
    }

    private Command LightAuto() {
        return Commands.run(() ->
                Light.getInstance().setState(Light.STATE.AUTO), Light.getInstance()).ignoringDisable(true);
    }

    private Command ManualShoot() {
        return new ShootManualCommand(shooter, arm, indicator);
    }
}
