package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterIOTalonFX implements ShooterIO {
    private final TalonFX leftShooterTalon = new TalonFX(LEFT_SHOOTER_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    private final TalonFX rightShooterTalon = new TalonFX(RIGHT_SHOOTER_MOTOR_ID,
            Constants.RobotConstants.CAN_BUS_NAME);
    private final TalonFX armTalon = new TalonFX(ARM_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    private final TalonFX pullerTalon = new TalonFX(PULLER_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    private final StatusSignal<Double> leftShooterVelocity = leftShooterTalon.getVelocity();
    private final StatusSignal<Double> leftShooterPosition = leftShooterTalon.getPosition();
    private final StatusSignal<Double> leftShooterAppliedVoltage = leftShooterTalon.getMotorVoltage();
    private final StatusSignal<Double> leftShooterSupplyCurrent = leftShooterTalon.getSupplyCurrent();
    private final StatusSignal<Double> rightShooterVelocity = rightShooterTalon.getVelocity();
    private final StatusSignal<Double> rightShooterPosition = rightShooterTalon.getPosition();
    private final StatusSignal<Double> rightShooterAppliedVoltage = rightShooterTalon.getMotorVoltage();
    private final StatusSignal<Double> rightShooterSupplyCurrent = rightShooterTalon.getSupplyCurrent();
    private final StatusSignal<Double> armPosition = armTalon.getPosition();
    private final StatusSignal<Double> armAppliedVoltage = armTalon.getMotorVoltage();
    private final StatusSignal<Double> armSupplyCurrent = armTalon.getSupplyCurrent();
    private final StatusSignal<Double> armTorqueCurrent = armTalon.getTorqueCurrent();
    private final StatusSignal<Double> pullerPosition = pullerTalon.getPosition();
    private final StatusSignal<Double> pullerAppliedVoltage = pullerTalon.getMotorVoltage();
    private final StatusSignal<Double> pullerSupplyCurrent = pullerTalon.getSupplyCurrent();
    private final StatusSignal<Double> pullerTorqueCurrent = pullerTalon.getTorqueCurrent();
    private boolean homed = false;
    private double targetShooterVelocity = 0;
    private double targetArmPosition = 0;

    public ShooterIOTalonFX() {
        var armMotorConfig = new TalonFXConfiguration()
                .withSlot0(armGainsUp)
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withClosedLoopRamps(rampConfigs)
                .withFeedback(feedbackConfigs);
        var response = armTalon.getConfigurator().apply(armMotorConfig);
        if (response.isError())
            System.out.println("Shooter Arm TalonFX failed config with error" + response);
        armTalon.setPosition(0);
        var pullerMotorConfig = new TalonFXConfiguration()
                .withFeedback(pullerfeedbackConfigs);
        pullerTalon.setPosition(0);
        response = pullerTalon.getConfigurator().apply(pullerMotorConfig);
        if (response.isError())
            System.out.println("Puller TalonFX failed config with error" + response);
        response = armTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Shooter Arm TalonFX failed sticky fault clearing with error" + response);
        response = pullerTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Puller TalonFX failed sticky fault clearing with error" + response);
        var shooterMotorConfig = new TalonFXConfiguration();
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
        shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        shooterMotorConfig.Feedback.SensorToMechanismRatio = 1;
        rightShooterTalon.getConfigurator().apply(shooterMotorConfig);
        response = leftShooterTalon.getConfigurator().apply(shooterMotorConfig);
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
        var config = new Slot0Configs();
        config.kP = 0.0031869;
        config.kI = 0;
        config.kD = 0;
        config.kS = 0.14926 * 6.28 * 2;
        config.kV = 0.0029813 * 6.28 * 2;
        config.kA = 0.0002347 * 6.28 * 2;
        leftShooterTalon.getConfigurator().apply(config);
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
                rightShooterSupplyCurrent,
                armPosition,
                armAppliedVoltage,
                armSupplyCurrent,
                armTorqueCurrent,
                pullerPosition,
                pullerAppliedVoltage,
                pullerSupplyCurrent,
                pullerTorqueCurrent);

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

        inputs.armPosition = Radians.of(Units.rotationsToRadians(armPosition.getValueAsDouble()));
        inputs.armAppliedVoltage = Volts.of(armAppliedVoltage.getValueAsDouble());
        inputs.armSupplyCurrent = Amps.of(armSupplyCurrent.getValueAsDouble());
        inputs.armTorqueCurrent = Amps.of(armTorqueCurrent.getValueAsDouble());

        inputs.pullerPosition = Radians.of(Units.rotationsToRadians(pullerPosition.getValueAsDouble()));
        inputs.pullerAppliedVoltage = Volts.of(pullerAppliedVoltage.getValueAsDouble());
        inputs.pullerSupplyCurrent = Amps.of(pullerSupplyCurrent.getValueAsDouble());
        inputs.pullerTorqueCurrent = Amps.of(pullerTorqueCurrent.getValueAsDouble());

        inputs.homed = homed;
        Slot0Configs newconfig = ShooterConstants.armGainsUp
                .withKP(Constants.ArmConstants.ArmP.get())
                .withKI(Constants.ArmConstants.ArmI.get())
                .withKD(Constants.ArmConstants.ArmD.get());
        armTalon.getConfigurator().apply(newconfig);

        inputs.targetShooterVelocity = RadiansPerSecond.of(targetShooterVelocity);
        inputs.targetArmPosition = Radians.of(targetArmPosition);
    }

    @Override
    public void setFlyWheelVoltage(Measure<Voltage> volts) {
        double rpm = volts.magnitude() / 12 * 6380;
        setFlyWheelVelocity(
                rpm
        );
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
    public void setArmVoltage(Measure<Voltage> volts) {
        armTalon.setControl(new VoltageOut(volts.magnitude()));
    }

    @Override
    public void setPullerVoltage(Measure<Voltage> volts) {
        pullerTalon.setControl(new VoltageOut(volts.magnitude()));
    }

    @Override
    public void setArmHome(Measure<Angle> rad) {
        armTalon.setPosition(rad.in(Rotations));
    }

    @Override
    public void setArmPosition(Measure<Angle> rad) {
        armTalon.setControl(new MotionMagicVoltage(rad.in(Rotations)));
        targetArmPosition = rad.in(Radians);
    }

    @Override
    public void setArmPosition(Measure<Angle> rad, boolean update) {
        if (update) {
            targetArmPosition = rad.in(Radians);
        }
        armTalon.setControl(new MotionMagicVoltage(rad.in(Rotations)));
    }

    @Override
    public void setArmBrakeMode(boolean isCoast) {
        var config = new MotorOutputConfigs();
        config.NeutralMode = isCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        armTalon.getConfigurator().apply(config);
        armTalon.setControl(new NeutralOut());
    }

    @Override
    public void setPullerBrakeMode(boolean isCoast) {
        var config = new MotorOutputConfigs();
        config.NeutralMode = isCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        pullerTalon.getConfigurator().apply(config);
        pullerTalon.setControl(new NeutralOut());
    }

    @Override
    public boolean setArmConfig(double p, double i, double d) {
        TalonFXConfiguration config = new TalonFXConfiguration()
                .withSlot0(new Slot0Configs().withKP(p).withKI(i).withKD(d))
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withClosedLoopRamps(rampConfigs)
                .withFeedback(feedbackConfigs);
        var response = armTalon.getConfigurator().apply(config);
        return response.isError();
    }

    @Override
    public void setHomed(boolean homed) {
        this.homed = homed;
    }
}
