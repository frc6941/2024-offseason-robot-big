package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;

import static edu.wpi.first.units.Units.Radians;

public class ResetArmAutoCommand extends Command {
    private final ArmSubsystem armSubsystem;

    public ResetArmAutoCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        armSubsystem.getIo().setArmHome(Radians.zero());
        armSubsystem.getIo().setHomed(true);
        // FIXME: wtf?
        Constants.armPosition = Radians.of(0);
    }
}
