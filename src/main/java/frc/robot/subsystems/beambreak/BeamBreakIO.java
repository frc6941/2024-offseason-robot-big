package frc.robot.subsystems.beambreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
    @AutoLog
    class BeamBreakIOInputs {
        public boolean isIntakerBeamBreakOn;
        public boolean isIndexerBeamBreakOn;
        public boolean isShooterBeamBreakOn;
        public boolean indexHasNote;
    }

    void updateInputs(BeamBreakIOInputs inputs);
}
