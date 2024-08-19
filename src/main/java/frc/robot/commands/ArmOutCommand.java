package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmOutCommand extends Command {
    ArmSubsystem armSubsystem;

    public ArmOutCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void execute() {
        armSubsystem.getIo().setArmPosition(Constants.ArmConstants.armOutPosition);
    }

    @Override
    public void end(boolean interrupted) {
        new ResetArmHomeCommand(armSubsystem).schedule();
    }
}
