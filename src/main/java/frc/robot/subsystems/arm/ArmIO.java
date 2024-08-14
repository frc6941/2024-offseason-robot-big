package frc.robot.subsystems.arm;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ArmConstants.armGainsClass.*;
import static frc.robot.Constants.ArmConstants.pullerGainsClass.*;

public interface ArmIO {
    void updateInputs(ArmIOInputs inputs);

    void setArmVoltage(Measure<Voltage> volts);

    void setPullerVoltage(Measure<Voltage> volts);

    void setPullerVelocity(double RPM);

    void setArmHome(Measure<Angle> rad);

    void setHomed(boolean homed);

    void setArmPosition(Measure<Angle> rad);

    void setArmPosition(Measure<Angle> rad, boolean update);

    void setArmBrakeMode(boolean isCoast);

    void setPullerBrakeMode(boolean isCoast);

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
        public double targetPullerVelocity = 0;

        public double ArmKP = ARM_KP.get();
        public double ArmKI = ARM_KI.get();
        public double ArmKD = ARM_KD.get();
        public double ArmKA = ARM_KA.get();
        public double ArmKV = ARM_KV.get();
        public double ArmKS = ARM_KS.get();

        public double PullerKP = PULLER_KP.get();
        public double PullerKI = PULLER_KI.get();
        public double PullerKD = PULLER_KD.get();
        public double PullerKA = PULLER_KA.get();
        public double PullerKV = PULLER_KV.get();
        public double PullerKS = PULLER_KS.get();
    }
}
