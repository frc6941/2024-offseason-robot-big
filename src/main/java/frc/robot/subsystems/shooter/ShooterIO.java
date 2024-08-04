package frc.robot.subsystems.shooter;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ShooterIO {
    void updateInputs(ShooterIOInputs inputs);

    void setFlyWheelVoltage(Measure<Voltage> volts);

    void setFlyWheelVelocity(double velocityRPM, double ffVoltage);

    void setFlyWheelVelocity(double velocityRPM);

    void setArmVoltage(Measure<Voltage> volts);

    void setPullerVoltage(Measure<Voltage> volts);

    void setArmHome(Measure<Angle> rad);

    void setHomed(boolean homed);

    void setArmPosition(Measure<Angle> rad);

    void setArmPosition(Measure<Angle> rad, boolean update);

    void setArmBrakeMode(boolean isCoast);

    void setPullerBrakeMode(boolean isCoast);

    boolean setArmConfig(double p, double i, double d);

    void runVolts(double volts);

    @AutoLog
    class ShooterIOInputs {
        public boolean homed = false;

        public Measure<Velocity<Angle>> leftShooterVelocity = RadiansPerSecond.zero();
        public Measure<Angle> leftShooterPosition = Radians.zero();
        public Measure<Voltage> leftShooterAppliedVoltage = Volts.zero();
        public Measure<Current> leftShooterSupplyCurrent = Amps.zero();

        public Measure<Velocity<Angle>> rightShooterVelocity = RadiansPerSecond.zero();
        public Measure<Angle> rightShooterPosition = Radians.zero();
        public Measure<Voltage> rightShooterAppliedVoltage = Volts.zero();
        public Measure<Current> rightShooterSupplyCurrent = Amps.zero();

        public Measure<Angle> armPosition = Radians.zero();
        public Measure<Voltage> armAppliedVoltage = Volts.zero();
        public Measure<Current> armSupplyCurrent = Amps.zero();
        public Measure<Current> armTorqueCurrent = Amps.zero();

        public Measure<Angle> pullerPosition = Radians.zero();
        public Measure<Voltage> pullerAppliedVoltage = Volts.zero();
        public Measure<Current> pullerSupplyCurrent = Amps.zero();
        public Measure<Current> pullerTorqueCurrent = Amps.zero();

        public Measure<Velocity<Angle>> targetShooterVelocity = RadiansPerSecond.zero();
        public Measure<Angle> targetArmPosition = Radians.zero();
    }
}
