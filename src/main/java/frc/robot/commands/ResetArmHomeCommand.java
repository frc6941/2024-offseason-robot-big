package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

import static edu.wpi.first.units.Units.Radians;

public class ResetArmHomeCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private boolean flag = false;

    public ResetArmHomeCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        armSubsystem.getIo().setArmPosition(Radians.of(0.09));
    }

    @Override
    public void execute() {
        if (Math.abs(armSubsystem.getInputs().armPosition.magnitude() - 0.09) < 0.04) {
            armSubsystem.getIo().setHomed(false);
            flag = true;
        }
    }

    @Override
    public boolean isFinished() {
        Constants.armPosition = Radians.of(0);
        return armSubsystem.getInputs().homed && flag;
    }
}
