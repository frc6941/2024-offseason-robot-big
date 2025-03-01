package frc.robot.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ArmConstants.*;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX armTalon = new TalonFX(ARM_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    private final TalonFX pullerTalon = new TalonFX(PULLER_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    private final StatusSignal<Double> armPosition = armTalon.getPosition();
    private final StatusSignal<Double> armAppliedVoltage = armTalon.getMotorVoltage();
    private final StatusSignal<Double> armSupplyCurrent = armTalon.getSupplyCurrent();
    private final StatusSignal<Double> armTorqueCurrent = armTalon.getTorqueCurrent();
    private final StatusSignal<Double> pullerPosition = pullerTalon.getPosition();
    private final StatusSignal<Double> pullerAppliedVoltage = pullerTalon.getMotorVoltage();
    private final StatusSignal<Double> pullerSupplyCurrent = pullerTalon.getSupplyCurrent();
    private final StatusSignal<Double> pullerTorqueCurrent = pullerTalon.getTorqueCurrent();
    private boolean homed = false;
    private double targetArmPosition = 0;
    private double targetPullerVelocity = 0;

    public ArmIOTalonFX() {
        //TODO: Arm PID
        var armMotorConfig = new TalonFXConfiguration()
                .withMotionMagic(motionMagicConfigs)
                .withMotorOutput(motorOutputConfigs)
                .withClosedLoopRamps(rampConfigs)//TODO: delete it
                .withFeedback(feedbackConfigs);
        var response = armTalon.getConfigurator().apply(armMotorConfig);
        if (response.isError())
            System.out.println("Arm TalonFX failed config with error" + response);
        armTalon.setPosition(0);
        var pullerMotorConfig = new TalonFXConfiguration()
                .withFeedback(pullerfeedbackConfigs);
        pullerTalon.setPosition(0);
        response = pullerTalon.getConfigurator().apply(pullerMotorConfig);
        if (response.isError())
            System.out.println("Puller TalonFX failed config with error" + response);
        response = armTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Arm TalonFX failed sticky fault clearing with error" + response);
        response = pullerTalon.clearStickyFaults();
        if (response.isError())
            System.out.println("Puller TalonFX failed sticky fault clearing with error" + response);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                armPosition,
                armAppliedVoltage,
                armSupplyCurrent,
                armTorqueCurrent,
                pullerPosition,
                pullerAppliedVoltage,
                pullerSupplyCurrent,
                pullerTorqueCurrent);

        inputs.armPosition = Radians.of(Units.rotationsToRadians(armPosition.getValueAsDouble()));
        inputs.armAppliedVoltage = Volts.of(armAppliedVoltage.getValueAsDouble());
        inputs.armSupplyCurrent = Amps.of(armSupplyCurrent.getValueAsDouble());
        inputs.armTorqueCurrent = Amps.of(armTorqueCurrent.getValueAsDouble());

        inputs.pullerPosition = Radians.of(Units.rotationsToRadians(pullerPosition.getValueAsDouble()));
        inputs.pullerAppliedVoltage = Volts.of(pullerAppliedVoltage.getValueAsDouble());
        inputs.pullerSupplyCurrent = Amps.of(pullerSupplyCurrent.getValueAsDouble());
        inputs.pullerTorqueCurrent = Amps.of(pullerTorqueCurrent.getValueAsDouble());

        inputs.homed = homed;
        inputs.targetArmPosition = Radians.of(targetArmPosition);
        inputs.targetPullerVelocity = targetPullerVelocity;

        armTalon.getConfigurator().apply(new Slot0Configs()
                .withKP(inputs.ArmKP)
                .withKI(inputs.ArmKI)
                .withKD(inputs.ArmKD)
                .withKA(inputs.ArmKA)
                .withKV(inputs.ArmKV)
                .withKS(inputs.ArmKS));

        pullerTalon.getConfigurator().apply(new Slot0Configs()
                .withKP(inputs.PullerKP)
                .withKI(inputs.PullerKI)
                .withKD(inputs.PullerKD)
                .withKA(inputs.PullerKA)
                .withKV(inputs.PullerKV)
                .withKS(inputs.PullerKS));
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
    public void setPullerVelocity(double RPM) {
        var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(RPM);
        pullerTalon.setControl(new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec),
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));
        targetPullerVelocity = velocityRadPerSec;
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
    public void setHomed(boolean homed) {
        this.homed = homed;
    }
}
