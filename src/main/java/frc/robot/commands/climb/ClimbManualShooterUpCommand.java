package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class ClimbManualShooterUpCommand extends Command {
    private final ArmSubsystem armSubsystem;

    public ClimbManualShooterUpCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        armSubsystem.getIo()
                .setPullerBrakeMode(true);
    }

    @Override
    public void execute() {
        var voltage = Constants.ArmConstants.armUpDownVoltage.mutableCopy().negate();
        armSubsystem.getIo()
                .setPullerVoltage(Volts.zero());
        armSubsystem.getIo()
                .setArmVoltage(voltage);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.getIo()
                .setPullerBrakeMode(false);
        armSubsystem.getIo().setArmVoltage(Volts.zero());
    }
}
