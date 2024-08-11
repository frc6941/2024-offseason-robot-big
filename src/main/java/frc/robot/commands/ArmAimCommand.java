package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Radians;

public class ArmAimCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final ShootingDecider shootingDecider;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;

    public ArmAimCommand(
            ArmSubsystem armSubsystem,
            Supplier<ShootingDecider.Destination> destinationSupplier
    ) {
        this.shootingDecider = ShootingDecider.getInstance();
        this.armSubsystem = armSubsystem;
        this.destinationSupplier = destinationSupplier;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armSubsystem.getIo().setArmPosition(Radians.of(
                Units.degreesToRadians(
                        shootingDecider.getShootingParameter(
                                destinationSupplier.get(),
                                Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)
                        ).getShootingAngle())));
    }


    @Override
    public void end(boolean interrupted) {
        armSubsystem.getIo().setArmPosition(Radians.zero());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

