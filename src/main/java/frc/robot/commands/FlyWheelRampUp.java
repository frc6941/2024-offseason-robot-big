package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.shooting.ShootingDecider;

import static frc.robot.Constants.ShooterConstants.defaultShootRPM;

import java.util.function.Supplier;

public class FlyWheelRampUp extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;


    public FlyWheelRampUp(
        ShooterSubsystem shooterSubsystem,
        Supplier<ShootingDecider.Destination> destinationSupplier

    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.destinationSupplier = destinationSupplier;

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double distance = Limelight.getInstance().getSpeakerRelativePosition().getNorm();

        SmartDashboard.putNumber("shooter desired angle", Units.degreesToRadians(
                shooterSubsystem.getInputs().leftShooterVelocity.magnitude()));

        shooterSubsystem.getIo().setFlyWheelVelocity(parameter.getVelocity());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setFlyWheelDirectVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }
}
