package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

import static edu.wpi.first.units.Units.Degrees;

public class ArmUpCommand extends Command {
    private final ArmSubsystem armSubsystem;

    public ArmUpCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        armSubsystem.getIo()
                .setPullerBrakeMode(true);
    }

    @Override
    public void execute() {
        Constants.armPosition = Constants.armPosition.plus(Degrees.of(0.1));
        armSubsystem.getIo()
                .setArmPosition(Constants.armPosition);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.getIo()
                .setPullerBrakeMode(false);
    }
}
