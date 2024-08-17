package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.utils.Utils;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

public class ClimbArmUpCommand extends Command {
    private final ArmSubsystem armSubsystem;

    public ClimbArmUpCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        Utils.armStartClimb = true;
    }

    @Override
    public void execute() {
        armSubsystem.getIo()
                .setPullerVoltage(Volts.zero());
        armSubsystem.getIo()
                .setArmPosition(Radians.of(2.8));
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.getIo().setArmVoltage(Volts.zero());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
