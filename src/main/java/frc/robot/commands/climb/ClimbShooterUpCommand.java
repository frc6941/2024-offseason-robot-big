package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.utils.Utils.armReachedClimb;

public class ClimbShooterUpCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final Timer timer = new Timer();

    public ClimbShooterUpCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        armSubsystem.getIo()
                .setPullerBrakeMode(true);
        timer.restart();
    }

    @Override
    public void execute() {
        var voltage = Constants.ArmConstants.armUpDownVoltage.mutableCopy().negate();
        if (armSubsystem.getInputs().armPosition.minus(Radians.of(2.52)).gt(Radians.of(0.04))) {
            voltage = Volts.zero();
            armReachedClimb = true;
        }
        if (!timer.hasElapsed(0.3)) {
            armSubsystem.getIo()
                    .setPullerVoltage(Constants.ArmConstants.pullVoltage);
        } else {
            armSubsystem.getIo()
                    .setPullerVoltage(Volts.zero());
            armSubsystem.getIo()
                    .setArmVoltage(voltage);
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.getIo()
                .setPullerBrakeMode(false);
        armSubsystem.getIo().setArmVoltage(Volts.zero());
    }

    @Override
    public boolean isFinished() {
        return armReachedClimb;
    }
}
