package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Radians;

public class ResetArmCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ResetArmCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.getIo().setHomed(false);
    }

    @Override
    public boolean isFinished() {
        Constants.armPosition = Radians.of(0);
        return shooterSubsystem.getInputs().homed;
    }
}
