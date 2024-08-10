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
import frc.robot.utils.shooting.ShootingDecider;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class ChassisAimCommand extends Command {
    private final Swerve Swerve;
    private final DoubleSupplier driverX;
    private final DoubleSupplier driverY;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;


    LinearFilter filter = LinearFilter.singlePoleIIR(0.2, 0.02);

    public ChassisAimCommand(
            Swerve Swerve,
            Supplier<ShootingDecider.Destination> destinationSupplier,
            DoubleSupplier driverX,
            DoubleSupplier driverY
            ) {
        this.Swerve = Swerve;
        this.driverX = driverX;
        this.driverY = driverY;
        this.destinationSupplier = destinationSupplier;
        this.shootingDecider = ShootingDecider.getInstance();


    }

    @Override
    public void initialize() {
        filter.reset();
    }

    @Override
    public void execute() {
        Swerve.drive(
                new Translation2d(
                        -driverX.getAsDouble() * Constants.SwerveConstants.maxSpeed.magnitude(),
                        -driverY.getAsDouble() * Constants.SwerveConstants.maxSpeed.magnitude()),
                0,
                true,
                false);
        filter.calculate(
            Units.degreesToRadians(
                shootingDecider.getShootingParameter(
                    destinationSupplier.get(), 
                    Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)
                    ).getFieldAimignAngle().getDegrees()));

        
        Swerve.setLockHeading(true);
        Swerve.setHeadingTarget(filter.lastValue());
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
