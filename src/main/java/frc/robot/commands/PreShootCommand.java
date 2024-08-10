package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.ShootingParameters.ShootingParameters;
import frc.robot.utils.ShootingParameters.SpeakerShootingParameters;

import static frc.robot.Constants.ShooterConstants.defaultShootRPM;

public class PreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private double defaultRPM = defaultShootRPM;

    public PreShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        defaultRPM = defaultShootRPM;
    }

    @Override
    public void execute() {

        double distance = Limelight.getInstance().getSpeakerRelativePosition().getNorm();

        ShootingParameters parameter = SpeakerShootingParameters.getInstance().getParameters(distance);
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
