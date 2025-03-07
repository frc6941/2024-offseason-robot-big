package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.intaker.IntakerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import static edu.wpi.first.units.Units.Volts;

public class IntakeCommand extends Command {
    private final IntakerSubsystem intakerSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final ArmSubsystem armSubsystem;
    private boolean enabledBefore = false;

    public IntakeCommand(
            IntakerSubsystem intakerSubsystem,
            BeamBreakSubsystem beamBreakSubsystem,
            ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem) {
        this.intakerSubsystem = intakerSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.armSubsystem = armSubsystem;
        addRequirements(intakerSubsystem);
    }

    @Override
    public void initialize() {
        enabledBefore = false;
    }

    @Override
    public void execute() {
        if (beamBreakSubsystem.getInputs().isIntakerBeamBreakOn) {
            enabledBefore = true;
        }
        if ((armSubsystem.getInputs().armPosition.magnitude() > 0.1
                && beamBreakSubsystem.getInputs().isIntakerBeamBreakOn)) {
            intakerSubsystem.getIo().setIntakeVoltage(Volts.of(0));
        } else {
            intakerSubsystem.getIo().setIntakeVoltage(Constants.IntakerConstants.intakeVoltage);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakerSubsystem.getIo()
                .setIntakeVoltage(Volts.zero());
        shooterSubsystem.getIo()
                .setFlyWheelDirectVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }

    @Override
    public boolean isFinished() {
        return (beamBreakSubsystem.getInputs().isIndexerBeamBreakOn &&
                !beamBreakSubsystem.getInputs().isIntakerBeamBreakOn);
    }
}
