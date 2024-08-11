package frc.robot.subsystems.shooter;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.shooterGainsClass.*;

public interface ShooterIO {
    void updateInputs(ShooterIOInputs inputs);

    void setFlyWheelDirectVoltage(Measure<Voltage> volts);

    void setFlyWheelVelocity(double velocityRPM, double ffVoltage);

    void setFlyWheelVelocity(double velocityRPM);

    void runVolts(double volts);

    double getVelocity();


    @AutoLog
    class ShooterIOInputs {
        public Measure<Velocity<Angle>> leftShooterVelocity = RadiansPerSecond.zero();
        public Measure<Angle> leftShooterPosition = Radians.zero();
        public Measure<Voltage> leftShooterAppliedVoltage = Volts.zero();
        public Measure<Current> leftShooterSupplyCurrent = Amps.zero();

        public Measure<Velocity<Angle>> rightShooterVelocity = RadiansPerSecond.zero();
        public Measure<Angle> rightShooterPosition = Radians.zero();
        public Measure<Voltage> rightShooterAppliedVoltage = Volts.zero();
        public Measure<Current> rightShooterSupplyCurrent = Amps.zero();

        public Measure<Velocity<Angle>> targetShooterVelocity = RadiansPerSecond.zero();

        public double ShooterKP = SHOOTER_KP.get();
        public double ShooterKI = SHOOTER_KI.get();
        public double ShooterKD = SHOOTER_KD.get();
        public double ShooterKA = SHOOTER_KA.get();
        public double ShooterKV = SHOOTER_KV.get();
        public double ShooterKS = SHOOTER_KS.get();
    }
}
