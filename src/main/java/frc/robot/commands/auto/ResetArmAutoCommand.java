package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Radians;

public class ResetArmAutoCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ResetArmAutoCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        shooterSubsystem.getIo().setHomed(true);
        Constants.armPosition = Radians.of(0);
    }
}
