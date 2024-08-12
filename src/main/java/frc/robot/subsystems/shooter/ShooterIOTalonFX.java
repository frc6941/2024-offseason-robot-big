package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;
import static frc.robot.Constants.ShooterConstants.LEFT_SHOOTER_MOTOR_ID;
import static frc.robot.Constants.ShooterConstants.RIGHT_SHOOTER_MOTOR_ID;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leftShooterTalon = new TalonFX(LEFT_SHOOTER_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    private final TalonFX rightShooterTalon = new TalonFX(RIGHT_SHOOTER_MOTOR_ID,
            Constants.RobotConstants.CAN_BUS_NAME);
    private final StatusSignal<Double> leftShooterVelocity = leftShooterTalon.getVelocity();
    private final StatusSignal<Double> leftShooterPosition = leftShooterTalon.getPosition();
    private final StatusSignal<Double> leftShooterAppliedVoltage = leftShooterTalon.getMotorVoltage();
    private final StatusSignal<Double> leftShooterSupplyCurrent = leftShooterTalon.getSupplyCurrent();
    private final StatusSignal<Double> rightShooterVelocity = rightShooterTalon.getVelocity();
    private final StatusSignal<Double> rightShooterPosition = rightShooterTalon.getPosition();
    private final StatusSignal<Double> rightShooterAppliedVoltage = rightShooterTalon.getMotorVoltage();
    private final StatusSignal<Double> rightShooterSupplyCurrent = rightShooterTalon.getSupplyCurrent();
    private double targetShooterVelocity = 0;

    public ShooterIOTalonFX() {
        var shooterMotorConfig = new TalonFXConfiguration();
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        shooterMotorConfig.Feedback.SensorToMechanismRatio = 1;
        leftShooterTalon.getConfigurator().apply(shooterMotorConfig);
        var response = leftShooterTalon.getConfigurator().apply(shooterMotorConfig);
        if (response.isError())
            System.out.println("Left Shooter TalonFX failed config with error" + response);
        response = leftShooterTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Left Shooter TalonFX failed sticky fault clearing with error" + response);
        response = rightShooterTalon.getConfigurator().apply(shooterMotorConfig);
        if (response.isError())
            System.out.println("Right Shooter TalonFX failed config with error" + response);
        response = rightShooterTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Right Shooter TalonFX failed sticky fault clearing with error" + response);
        rightShooterTalon.setControl(new Follower(leftShooterTalon.getDeviceID(),
                true));
    }

    public void runVolts(double volts) {
        leftShooterTalon.setControl(new VoltageOut(volts));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                leftShooterVelocity,
                leftShooterPosition,
                leftShooterAppliedVoltage,
                leftShooterSupplyCurrent,
                rightShooterVelocity,
                rightShooterPosition,
                rightShooterAppliedVoltage,
                rightShooterSupplyCurrent);

        inputs.leftShooterVelocity = RadiansPerSecond
                .of(Units.rotationsToRadians(leftShooterVelocity.getValueAsDouble()));
        inputs.leftShooterPosition = Radians.of(Units.rotationsToRadians(leftShooterPosition.getValueAsDouble()));
        inputs.leftShooterAppliedVoltage = Volts.of(leftShooterAppliedVoltage.getValueAsDouble());
        inputs.leftShooterSupplyCurrent = Amps.of(leftShooterSupplyCurrent.getValueAsDouble());

        inputs.rightShooterVelocity = RadiansPerSecond
                .of(Units.rotationsToRadians(rightShooterVelocity.getValueAsDouble()));
        inputs.rightShooterPosition = Radians.of(Units.rotationsToRadians(rightShooterPosition.getValueAsDouble()));
        inputs.rightShooterAppliedVoltage = Volts.of(rightShooterAppliedVoltage.getValueAsDouble());
        inputs.rightShooterSupplyCurrent = Amps.of(rightShooterSupplyCurrent.getValueAsDouble());

        inputs.targetShooterVelocity = RadiansPerSecond.of(targetShooterVelocity);

        leftShooterTalon.getConfigurator().apply(new Slot0Configs()
                .withKP(inputs.ShooterKP)
                .withKI(inputs.ShooterKI)
                .withKD(inputs.ShooterKD)
                .withKA(inputs.ShooterKA)
                .withKV(inputs.ShooterKV)
                .withKS(inputs.ShooterKS));
    }

    @Override
    public void setFlyWheelDirectVoltage(Measure<Voltage> volts) {
        leftShooterTalon.setControl(new VoltageOut(volts.magnitude()));
    }

    @Override
    public void setFlyWheelVelocity(double velocityRPM) {
        var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        leftShooterTalon.setControl(new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec),
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));
        targetShooterVelocity = velocityRadPerSec;
    }

    @Override
    public void setFlyWheelVelocity(double velocityRPM, double ffVoltage) {
        var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        leftShooterTalon.setControl(new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec),
                0.0,
                true,
                ffVoltage,
                0,
                false,
                false,
                false
        ));
        targetShooterVelocity = velocityRadPerSec;
    }

    @Override
    public double getVelocity() {
        return rightShooterVelocity.getValueAsDouble() * 60;
    }

}
