package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.ShootingParameters.ShootingParameters;
import frc.robot.utils.ShootingParameters.SpeakerShootingParameters;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class ChassisAimCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;
    private final Swerve Swerve;
    private final DoubleSupplier driverX;
    private final DoubleSupplier driverY;
    LinearFilter filter = LinearFilter.singlePoleIIR(0.2, 0.02);
    private LoggedDashboardNumber distanceLogged = new LoggedDashboardNumber("Distance");

    public ChassisAimCommand(
            ShooterSubsystem shooterSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            Swerve Swerve,
            DoubleSupplier driverX,
            DoubleSupplier driverY) {
        this.shooterSubsystem = shooterSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
        this.Swerve = Swerve;
        this.driverX = driverX;
        this.driverY = driverY;
    }

    @Override
    public void initialize() {
        filter.reset();
    }

    @Override
    public void execute() {
//        if (!beamBreakSubsystem.isIntakeReady()) {
//            shooterSubsystem.getIo().setArmPosition(Radians.zero(), false);
//            return;
//        }
        if (Limelight.getInstance().getSpeakerRelativePosition().getNorm() >
                SpeakerShootingParameters.getInstance().getFarthestDistance()/*+0.5*/) {
            shooterSubsystem.getIo().setArmPosition(Radians.zero(), false);
            Swerve.setLockHeading(true);
            filter.calculate(Limelight.getInstance().getSpeakerRelativePosition().getAngle().getDegrees());
            Swerve.setHeadingTarget(filter.lastValue());
            return;
        }
        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);

        var distance = Limelight.getInstance().getSpeakerRelativePosition().getNorm();

        ShootingParameters parameter = SpeakerShootingParameters.getInstance().getParameters(distance);
        distanceLogged.set(distance);
        Swerve.drive(
                new Translation2d(
                        -driverX.getAsDouble() * Constants.SwerveConstants.maxSpeed.magnitude(),
                        -driverY.getAsDouble() * Constants.SwerveConstants.maxSpeed.magnitude()),
                0,
                true,
                false);
        filter.calculate(Limelight.getInstance().getSpeakerRelativePosition().getAngle().getDegrees());
        Swerve.setLockHeading(true);
        Swerve.setHeadingTarget(filter.lastValue());
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.setLockHeading(false);
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
