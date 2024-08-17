package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.utils.Utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

/**
 * Climbs forever without stopping.
 * Used in endgame.
 */
public class ClimbPullerDownCommand extends Command {
    private final ArmSubsystem armSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;

    public ClimbPullerDownCommand(
            ArmSubsystem armSubsystem,
            IndicatorSubsystem indicatorSubsystem
    ) {
        this.armSubsystem = armSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        addRequirements(armSubsystem);
    }


    @Override
    public void initialize() {
        if (!Utils.armStartClimb) return;
        armSubsystem.getIo().setArmVoltage(Volts.zero());
        armSubsystem.getIo().setArmBrakeMode(true);
        armSubsystem.getIo().setPullerBrakeMode(false);
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.CLIMBING);
    }

    @Override
    public void execute() {
        if (armSubsystem.getInputs().armPosition.lt(Degrees.of(15))) {
            armSubsystem.getIo().setPullerVelocity(0);
        } else {
            armSubsystem.getIo().setPullerVelocity(Constants.ArmConstants.pullVelocity);
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.getIo().setArmBrakeMode(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
