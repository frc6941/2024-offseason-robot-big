package frc.robot.subsystems.indexer;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IndexerIO {
    void updateInputs(IndexerIOInputs inputs);

    void setIndexRPM(double velocityRPM);

    @AutoLog
    class IndexerIOInputs {
        public Measure<Velocity<Angle>> indexVelocity = RadiansPerSecond.zero();
        public Measure<Angle> indexPosition = Radians.zero();
        public double indexAppliedRPM = 0;
        public Measure<Current> indexSupplyCurrent = Amps.zero();
    }
}
