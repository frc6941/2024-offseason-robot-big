package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ChassisAimCommand extends Command {
    private final Swerve swerve;
    private final DoubleSupplier driverX;
    private final DoubleSupplier driverY;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;

    LinearFilter filter = LinearFilter.singlePoleIIR(0.2, 0.02);

    public ChassisAimCommand(
            Swerve Swerve,
            Supplier<ShootingDecider.Destination> destinationSupplier,
            DoubleSupplier driverX,
            DoubleSupplier driverY) {
        this.swerve = Swerve;
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
        swerve.drive(
                new Translation2d(
                        -driverX.getAsDouble() * Constants.SwerveConstants.maxSpeed.magnitude(),
                        -driverY.getAsDouble() * Constants.SwerveConstants.maxSpeed.magnitude()),
                0,
                true,
                false);

        swerve.setLockHeading(true);
        swerve.setHeadingTarget(filter.calculate(
                shootingDecider.getShootingParameter(
                        destinationSupplier.get(),
                        swerve.getLocalizer().getCoarseFieldPose(0)).getFieldAimingAngle().getDegrees()));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
