package frc.robot.commands.climb;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

public class ClimbCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final boolean reverse;

    public ClimbCommand(
            ArmSubsystem armSubsystem,
            boolean reverse
    ) {
        this.armSubsystem = armSubsystem;
        this.reverse = reverse;
    }


    @Override
    public void initialize() {
        armSubsystem.getIo().setArmBrakeMode(true);
    }

    @Override
    public void execute() {
        Measure<Voltage> pullVoltage;
        if (reverse) {
            pullVoltage = Constants.ArmConstants.pullVoltage.negate();
        } else {
            pullVoltage = Constants.ArmConstants.pullVoltage;
        }
        if (armSubsystem.getInputs().pullerTorqueCurrent.gt(Amps.of((50)))) {
            pullVoltage = Volts.zero();
        }
        armSubsystem.getIo().setPullerVoltage(pullVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.getIo().setArmBrakeMode(false);
        armSubsystem.getIo()
                .setPullerVoltage(Volts.zero());
    }
}
