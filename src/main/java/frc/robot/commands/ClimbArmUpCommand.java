package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

public class ClimbArmUpCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final Timer timer = new Timer();

    public ClimbArmUpCommand(ArmSubsystem armSubsystem) {
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
        if (!timer.hasElapsed(0.3)) {
            armSubsystem.getIo()
                    .setPullerVoltage(Constants.ArmConstants.pullVoltage);
        } else {
            armSubsystem.getIo()
                    .setPullerVoltage(Volts.zero());
            armSubsystem.getIo()
                    .setArmPosition(Radians.of(2.6));
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.getIo()
                .setPullerBrakeMode(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
