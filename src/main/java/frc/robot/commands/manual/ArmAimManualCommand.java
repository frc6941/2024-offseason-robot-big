package frc.robot.commands.manual;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.ResetArmHomeCommand;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.shooting.ShootingDecider;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Radians;

public class ArmAimManualCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final ShootingDecider shootingDecider;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;

    public ArmAimManualCommand(
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
                                new Pose2d(
                                        AllianceFlipUtil.apply(Constants.FieldConstants.Speaker.centerSpeakerOpening
                                                .minus(new Translation3d(0.4, 0, 0))).toTranslation2d(),
                                        Rotation2d.fromDegrees(0))
                        ).getShootingAngle())));
    }


    @Override
    public void end(boolean interrupted) {
        new ResetArmHomeCommand(armSubsystem).schedule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

