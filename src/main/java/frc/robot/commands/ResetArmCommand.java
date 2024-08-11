package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

import static edu.wpi.first.units.Units.Radians;

public class ResetArmCommand extends Command {
    private final ArmSubsystem armSubsystem;

    public ResetArmCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        armSubsystem.getIo().setHomed(false);
    }

    @Override
    public boolean isFinished() {
        Constants.armPosition = Radians.of(0);
        return armSubsystem.getInputs().homed;
    }
}
