package frc.robot.subsystems.indexer;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IndexerConstants.indexerGainsClass.*;

public interface IndexerIO {
    void updateInputs(IndexerIOInputs inputs);

    void setIndexRPM(double velocityRPM);

    @AutoLog
    class IndexerIOInputs {
        public Measure<Velocity<Angle>> indexVelocity = RadiansPerSecond.zero();
        public Measure<Angle> indexPosition = Radians.zero();
        public double indexAppliedRPM = 0;
        public Measure<Current> indexSupplyCurrent = Amps.zero();
        public double IndexerKP = INDEXER_KP.get();
        public double IndexerKI = INDEXER_KI.get();
        public double IndexerKD = INDEXER_KD.get();
        public double IndexerKA = INDEXER_KA.get();
        public double IndexerKV = INDEXER_KV.get();
        public double IndexerKS = INDEXER_KS.get();
    }
}
