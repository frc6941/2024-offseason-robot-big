package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.display.OperatorDashboard;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

@Getter
public class BeamBreakSubsystem extends SubsystemBase {
    private final BeamBreakIO io;
    private final BeamBreakIOInputsAutoLogged inputs = new BeamBreakIOInputsAutoLogged();
    private boolean lastRecordedState;
    private boolean noteState = false;

    public BeamBreakSubsystem(BeamBreakIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        OperatorDashboard.getInstance().updateNotePathStatus(
                inputs.isIntakerBeamBreakOn,
                inputs.isIndexerBeamBreakOn,
                inputs.isShooterBeamBreakOn);

        if (!lastRecordedState && inputs.isIndexerBeamBreakOn) {
            noteState = true;
        }
        lastRecordedState = inputs.isIndexerBeamBreakOn;

        io.updateInputs(inputs);
        Logger.processInputs("Beam Break", inputs);
    }

    public boolean hasNote() {
        return noteState;
    }

    public void noteCleared() {
        noteState = false;
    }

    public boolean isIntakeReady() {
        return inputs.isIndexerBeamBreakOn &&
                !inputs.isIntakerBeamBreakOn;
    }
}
