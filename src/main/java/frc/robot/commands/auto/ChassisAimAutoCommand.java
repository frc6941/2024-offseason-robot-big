package frc.robot.commands.auto;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;

import java.util.function.Supplier;

public class ChassisAimAutoCommand extends Command {
    private final Swerve swerve;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;

    LinearFilter filter = LinearFilter.singlePoleIIR(0.2, 0.02);

    public ChassisAimAutoCommand(
            Swerve Swerve,
            Supplier<ShootingDecider.Destination> destinationSupplier) {
        this.swerve = Swerve;
        this.destinationSupplier = destinationSupplier;
        this.shootingDecider = ShootingDecider.getInstance();
    }

    @Override
    public void initialize() {
        filter.reset();
        swerve.setLockHeading(true);
    }

    @Override
    public void execute() {
//        swerve.autoDrive(
//                new Translation2d(0, 0),
//                0,
//                true,
//                false);
        swerve.setHeadingTarget(filter.calculate(
                shootingDecider.getShootingParameter(
                                destinationSupplier.get(),
                                swerve.getLocalizer().getCoarseFieldPose(0))
                        .getFieldAimingAngle().getDegrees()));
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
