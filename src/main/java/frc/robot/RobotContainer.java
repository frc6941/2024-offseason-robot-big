// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
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
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
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

        indicator.setDefaultCommand(Commands.run(() -> indicator.setPattern(IndicatorIO.Patterns.NORMAL), indicator));
    }

    private void configureAuto() {
        NamedCommands.registerCommand("AutoShoot", speakerAutoShot().withTimeout(2.0));
        NamedCommands.registerCommand("Intake", intake());
        NamedCommands.registerCommand("ResetArm", new ResetArmCommand(arm));
        NamedCommands.registerCommand("FlyWheelRampUp", new FlyWheelRampUp(shooter, () -> Destination.SPEAKER));//READ ME change all "Preshoot" in auto files
        NamedCommands.registerCommand("AutoPreArm", new ArmAimCommand(arm, () -> Destination.SPEAKER));
        NamedCommands.registerCommand("ChassisAim", new ChassisAimCommand(swerve, () -> Destination.SPEAKER, () -> 0, () -> 0));
        NamedCommands.registerCommand("Shoot", new DeliverNoteCommand(indexer, beamBreak, indicator));


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
//        shooter.setDefaultCommand(Commands.runOnce(() -> {
//            shooter.getIo().setFlyWheelDirectVoltage(Constants.ShooterConstants.shooterConstantVoltage);
//            shooter.getIo().setArmPosition(Radians.zero());
//        }, shooter));

        driverController.start().onTrue(resetOdom());
        driverController.leftBumper().whileTrue(
                intake().andThen(rumbleDriver(1.0)));

        // superstructure
        driverController.x().whileTrue(
                ferryShot()
                        .alongWith(setDest(Destination.FERRY))
                        .andThen(rumbleDriver(1.0)));

        driverController.rightBumper().whileTrue(
                ampShot()
                        .alongWith(setDest(Destination.AMP))
                        .andThen(rumbleDriver(1.0)));
        driverController.b().whileTrue(

                speakerShot()
                        .alongWith(setDest(Destination.SPEAKER))
                        .andThen(rumbleDriver(1.0)));
        driverController.rightTrigger().whileTrue(ampAim());
        driverController.rightTrigger().onFalse(justShoot().withTimeout(0.3));
        driverController.leftTrigger().whileTrue(outtake());

        // operator superstructure commands
        operatorController.povRight().onTrue(setDest(Destination.FERRY));
        operatorController.povLeft().onTrue(setDest(Destination.SPEAKER));
        operatorController.a().toggleOnTrue(flyWheelOn());
        operatorController.povUp().onTrue(new ResetArmCommand(arm));


    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
        // return null;
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

    private Command flyWheelOn() {
        return new FlyWheelRampUp(shooter, () -> dashboard.getCurrDestination());
    }

    private Command selectShot() {
        return Commands.select(shootingCommandMapping, dashboard::getCurrDestination);
    }

    private Command facing(double fieldAngleDeg) {
        return new SetFacingCommand(swerve, fieldAngleDeg);
    }

    private Command intake() {
        return new IntakeCommand(intaker, beamBreak, indicator, shooter, arm)
                .alongWith(new IndexCommand(indexer, beamBreak));
    }

    private Command outtake() {
        return new IntakeOutCommand(intaker)
                .alongWith(new IndexOutCommand(indexer));
    }

    private Command drive() {
        return Commands.runOnce(() -> {
            Translation2d transVel = new Translation2d(
                    -driverController.getLeftY(),
                    -driverController.getLeftX()).times(Constants.SwerveConstants.maxSpeed.magnitude());
            double rotVel = -Constants.RobotConstants.driverController.getRightX()
                    * Constants.SwerveConstants.maxAngularRate.magnitude();
            swerve.drive(transVel, rotVel, true, false);
        }, swerve);
    }

    private Command resetOdom() {
        return Commands.runOnce(() -> {
            swerve.resetHeadingController();
            Rotation2d a = Rotation2d.fromDegrees(swerve.getLocalizer().getLatestPose().getRotation().getDegrees());
            Pose2d b = new Pose2d(new Translation2d(0, 0), a);
            swerve.resetPose(b);
        });
    }

    private Command setDest(Destination des) {
        return Commands.runOnce(() -> dashboard.updateDestination(des));
    }
}
