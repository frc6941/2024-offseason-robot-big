package frc.robot.commands;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.utils.Utils;

import static edu.wpi.first.units.Units.Amps;
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
    }


    @Override
    public void initialize() {
        armSubsystem.getIo().setArmBrakeMode(true);
        indicatorSubsystem.setPattern(IndicatorIO.Patterns.CLIMBING);
    }

    @Override
    public void execute() {
        Measure<Voltage> pullVoltage;
        pullVoltage = Constants.ArmConstants.pullVoltage.negate();
        if (armSubsystem.getInputs().pullerTorqueCurrent.gt(Amps.of((50)))) {
            pullVoltage = Volts.zero();
        }
        armSubsystem.getIo().setPullerVoltage(pullVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.getIo().setArmBrakeMode(false);
        Utils.armReachedClimb = false;
    }
}
