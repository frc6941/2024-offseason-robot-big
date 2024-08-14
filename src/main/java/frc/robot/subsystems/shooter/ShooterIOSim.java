package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class ShooterIOSim implements ShooterIO {
    private static final double LOOP_PERIOD_SECS = 0.02;
    private final DCMotorSim leftShooterTalonSim = new DCMotorSim(
            DCMotor.getFalcon500(1),
            6.75, 0.025
    );
    private final DCMotorSim rightShooterTalonSim = new DCMotorSim(
            DCMotor.getFalcon500(1),
            6.75, 0.025
    );

    private Measure<Voltage> leftShooterAppliedVoltage = Volts.zero();
    private Measure<Voltage> rightShooterAppliedVoltage = Volts.zero();
    private Measure<Velocity<Angle>> targetShooterVelocity = RadiansPerSecond.zero();

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftShooterTalonSim.update(LOOP_PERIOD_SECS);
        rightShooterTalonSim.update(LOOP_PERIOD_SECS);

        inputs.leftShooterVelocity =
                RadiansPerSecond.of(leftShooterTalonSim.getAngularVelocityRadPerSec());
        inputs.leftShooterPosition =
                Radians.of(leftShooterTalonSim.getAngularPositionRad());
        inputs.leftShooterAppliedVoltage =
                leftShooterAppliedVoltage;
        inputs.leftShooterSupplyCurrent =
                Amps.of(leftShooterTalonSim.getCurrentDrawAmps());

        inputs.rightShooterVelocity =
                RadiansPerSecond.of(rightShooterTalonSim.getAngularVelocityRadPerSec());
        inputs.rightShooterPosition =
                Radians.of(rightShooterTalonSim.getAngularPositionRad());
        inputs.rightShooterAppliedVoltage =
                rightShooterAppliedVoltage;
        inputs.rightShooterSupplyCurrent =
                Amps.of(rightShooterTalonSim.getCurrentDrawAmps());

        inputs.targetShooterVelocity = targetShooterVelocity;
    }

    @Override
    public void setFlyWheelDirectVoltage(Measure<Voltage> volts) {
        leftShooterAppliedVoltage = volts;
        rightShooterAppliedVoltage = volts;
        leftShooterTalonSim.setInputVoltage(volts.magnitude());
        rightShooterTalonSim.setInputVoltage(volts.magnitude());
    }

    @Override
    public void setFlyWheelVelocity(double velocityRPM, double ffVoltage) {
        setFlyWheelVelocity(velocityRPM);
    }

    @Override
    public void setFlyWheelVelocity(double velocityRPM) {
        leftShooterTalonSim.setState(0, velocityRPM);
        targetShooterVelocity = RadiansPerSecond.of(velocityRPM / 60);
    }

    @Override
    public void runVolts(double volts) {

    }

    @Override
    public double getVelocity() {
        return rightShooterTalonSim.getAngularVelocityRadPerSec() / 6.28 * 60;
    }

}
