package frc.robot.subsystems.beambreak;

import frc.robot.Constants;
import frc.robot.drivers.BeamBreak;

public class BeamBreakIORev implements BeamBreakIO {
    public final BeamBreak intakerBeamBreak =
            new BeamBreak(Constants.BeamBreakConstants.INTAKER_BEAM_BREAK_ID);
    public final BeamBreak indexerBeamBreak =
            new BeamBreak(Constants.BeamBreakConstants.INDEXER_BEAM_BREAK_ID);
    public final BeamBreak shooterBeamBreak =
            new BeamBreak(Constants.BeamBreakConstants.SHOOTER_BEAM_BREAK_ID);

    @Override
    public void updateInputs(BeamBreakIOInputs inputs) {
        inputs.isIntakerBeamBreakOn = intakerBeamBreak.get();
        inputs.isIndexerBeamBreakOn = indexerBeamBreak.get();
        inputs.isShooterBeamBreakOn = shooterBeamBreak.get();
    }
}
