package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreakIORev;
import frc.robot.subsystems.intaker.IntakerIOTalonFX;

import static edu.wpi.first.units.Units.Volts;

public class IntakeCommand extends Command {
    private final IntakerIOTalonFX intakerSubsystem;
    private final BeamBreakIORev beamBreakSubsystem;

    public IntakeCommand(
			IntakerIOTalonFX intakerSubsystem,
			BeamBreakIORev beamBreakSubsystem
    ) {
        this.intakerSubsystem = intakerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
    }

    @Override
    public void execute() {
        if (isFinished()) return;
        intakerSubsystem
                .setIntakeVoltage(Constants.IntakerConstants.intakeVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        intakerSubsystem
                .setIntakeVoltage(Volts.zero());
        if (interrupted) return;
    }


    @Override
    public boolean isFinished() {
        return beamBreakSubsystem.indexerBeamBreak.get() &&
                !beamBreakSubsystem.intakerBeamBreak.get();
    }
}
