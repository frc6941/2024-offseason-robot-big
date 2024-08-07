package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.ShootingParameters;
import frc.robot.utils.ShootingParametersTable;

import static frc.robot.Constants.ShooterConstants.defaultShootRPM;

public class PreShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private double defaltRPM = defaultShootRPM;

    public PreShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        defaltRPM = defaultShootRPM;
    }

    @Override
    public void execute() {

        var targetOptional = Limelight.getTarget();
        if (targetOptional.isEmpty()) {
            shooterSubsystem.getIo().setFlyWheelVelocity(defaultShootRPM);
            return;
        }

        var target = targetOptional.get();
        var distance = target
                .targetPoseCameraSpace()
                .getTranslation()
                .getDistance(new Translation3d());

        ShootingParameters parameter = ShootingParametersTable.getInstance().getParameters(distance);
        SmartDashboard.putNumber("shooter desired angle", Units.degreesToRadians(
                shooterSubsystem.getInputs().leftShooterVelocity.magnitude()));

        shooterSubsystem.getIo().setFlyWheelVelocity(parameter.getVoltage());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setFlyWheelDirectVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }
}
