package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;

public class ArmAimCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final ShootingDecider shootingDecider;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;

    public ArmAimCommand(
            ShooterSubsystem shooterSubsystem,
            Supplier<ShootingDecider.Destination> destinationSupplier
    ) {
        this.shootingDecider = ShootingDecider.getInstance();
        this.shooterSubsystem = shooterSubsystem;
        this.destinationSupplier = destinationSupplier;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {        
        shooterSubsystem.getIo().setArmPosition(Radians.of(
            Units.degreesToRadians(
                shootingDecider.getShootingParameter(
                    destinationSupplier.get(), 
                    Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)
                ).getShootingAngle())));
    }
    

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

