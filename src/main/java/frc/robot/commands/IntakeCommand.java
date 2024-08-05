package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intaker.IntakerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class IntakeCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private boolean enabledBefore = false;

    public IntakeCommand(
            IntakerSubsystem intakerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            ShooterSubsystem shooterSubsystem) {
        this.intakerSubsystem = intakerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        enabledBefore = false;
    }

    @Override
    public void execute() {
        if (isFinished())
            return;
        if (beamBreakSubsystem.getInputs().isIntakerBeamBreakOn) {
            enabledBefore = true;
        }
        if (enabledBefore && !beamBreakSubsystem.getInputs().isIntakerBeamBreakOn) {
            intakerSubsystem.getIo().setIntakeVoltage(Volts.of(0.5));
            return;
        }
        if ((shooterSubsystem.getInputs().armPosition.magnitude() > 0.1
                && beamBreakSubsystem.getInputs().isIntakerBeamBreakOn)) {
            intakerSubsystem.getIo().setIntakeVoltage(Volts.of(0));
        } else {
            intakerSubsystem.getIo().setIntakeVoltage(Constants.IntakerConstants.intakeVoltage);
        }

        shooterSubsystem.getIo().setFlyWheelVoltage(Volts.of(2));

        // debug(" " + shooterSubsystem.getInputs().armPosition.magnitude());
    }

    @Override
    public void end(boolean interrupted) {
        intakerSubsystem.getIo()
                .setIntakeVoltage(Volts.zero());
        shooterSubsystem.getIo()
                .setFlyWheelVoltage(Constants.ShooterConstants.shooterConstantVoltage);
        if (interrupted)
            return;
        indicatorSubsystem
                .setPattern(IndicatorIO.Patterns.FINISH_INTAKE);
    }

    @Override
    public boolean isFinished() {
        return (beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
                !beamBreakSubsystem.getInputs().isIntakerBeamBreakOn);
    }
}
