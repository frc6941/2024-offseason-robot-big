package frc.robot.subsystems.arm;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ArmIO {
    void updateInputs(ArmIOInputs inputs);

    void setArmVoltage(Measure<Voltage> volts);

    void setPullerVoltage(Measure<Voltage> volts);

    void setArmHome(Measure<Angle> rad);

    void setHomed(boolean homed);

    void setArmPosition(Measure<Angle> rad);

    void setArmPosition(Measure<Angle> rad, boolean update);

    void setArmBrakeMode(boolean isCoast);

    void setPullerBrakeMode(boolean isCoast);

    boolean setArmConfig(double p, double i, double d);

    @AutoLog
    class ArmIOInputs {
        public boolean homed = false;

        public Measure<Angle> armPosition = Radians.zero();
        public Measure<Voltage> armAppliedVoltage = Volts.zero();
        public Measure<Current> armSupplyCurrent = Amps.zero();
        public Measure<Current> armTorqueCurrent = Amps.zero();

        public Measure<Angle> pullerPosition = Radians.zero();
        public Measure<Voltage> pullerAppliedVoltage = Volts.zero();
        public Measure<Current> pullerSupplyCurrent = Amps.zero();
        public Measure<Current> pullerTorqueCurrent = Amps.zero();

        public Measure<Angle> targetArmPosition = Radians.zero();
    }
}
