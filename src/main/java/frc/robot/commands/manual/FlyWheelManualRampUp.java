package frc.robot.commands.manual;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.display.OperatorDashboard;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.shooting.ShootingDecider;

import java.util.function.Supplier;

public class FlyWheelManualRampUp extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;

    public FlyWheelManualRampUp(
            ShooterSubsystem shooterSubsystem,
            Supplier<ShootingDecider.Destination> destinationSupplier

    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.destinationSupplier = destinationSupplier;
        this.shootingDecider = ShootingDecider.getInstance();
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        OperatorDashboard.getInstance().updateFlyWheelOn(true);
        shooterSubsystem.getIo().setFlyWheelVelocity(
                shootingDecider.getShootingParameter(
                        destinationSupplier.get(),
                        new Pose2d(
                                AllianceFlipUtil.apply(Constants.FieldConstants.Speaker.centerSpeakerOpening
                                        .minus(new Translation3d(0.4, 0, 0))).toTranslation2d(),
                                Rotation2d.fromDegrees(0))
                ).getShootingVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setFlyWheelDirectVoltage(Constants.ShooterConstants.shooterConstantVoltage);
        OperatorDashboard.getInstance().updateFlyWheelOn(false);
    }
}
