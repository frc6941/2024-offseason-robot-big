package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;

import java.util.function.Supplier;

public class FlyWheelRampUp extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;

    public FlyWheelRampUp(
            ShooterSubsystem shooterSubsystem,
            Supplier<ShootingDecider.Destination> destinationSupplier

    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.destinationSupplier = destinationSupplier;
        this.shootingDecider = ShootingDecider.getInstance();

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooterSubsystem.getIo().setFlyWheelVelocity(
                shootingDecider.getShootingParameter(
                        destinationSupplier.get(),
                        Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)
        ).getShootingVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setFlyWheelDirectVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }
}
