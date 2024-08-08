package org.frcteam6941.swerve;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import lombok.Getter;

public class SimSwerveIO {
    public final SimSwerveModule module;
    private final TalonFX steerSim, driveSim;
    private final CANcoder CANcoder;
    private final double driveRotationsPerMeter;
    private final double couplingRatioDriveRotorToCANCoder;
    private final double speedAt12VoltsMps;
    private final SwerveModule.ClosedLoopOutputType steerClosedLoopOutput;
    private final SwerveModule.ClosedLoopOutputType driveClosedLoopOutput;
    @Getter
    private final SwerveModulePosition internalState = new SwerveModulePosition();
    private final MotionMagicVoltage angleVoltageSetter = new MotionMagicVoltage(0.0);
    private final MotionMagicTorqueCurrentFOC angleTorqueSetter = new MotionMagicTorqueCurrentFOC(0.0);
    private final MotionMagicExpoVoltage angleVoltageExpoSetter = new MotionMagicExpoVoltage(0.0);
    private final MotionMagicExpoTorqueCurrentFOC angleTorqueExpoSetter = new MotionMagicExpoTorqueCurrentFOC(0.0);
    private final VoltageOut voltageOpenLoopSetter = new VoltageOut(0.0);
    private final VelocityVoltage velocityVoltageSetter = new VelocityVoltage(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueSetter = (new VelocityTorqueCurrentFOC(0.0)).withOverrideCoastDurNeutral(true);
    private double sigDrivePosition;
    private double sigDriveVelocity;
    private double sigSteerPosition;
    private double sigSteerVelocity;

    public SimSwerveIO(SwerveModuleConstants constants) {
        module = new SimSwerveModule(constants.SteerMotorGearRatio, constants.SteerInertia, constants.SteerFrictionVoltage, constants.SteerMotorInverted, constants.DriveMotorGearRatio, constants.DriveInertia, constants.DriveFrictionVoltage, constants.DriveMotorInverted);
        steerSim = new TalonFX(constants.DriveMotorId);
        driveSim = new TalonFX(constants.SteerMotorId);
        CANcoder = new CANcoder(constants.CANcoderId);

        // constants
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 6.283185307179586 * Units.inchesToMeters(constants.WheelRadius);
        driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
        couplingRatioDriveRotorToCANCoder = constants.CouplingGearRatio;
        speedAt12VoltsMps = constants.SpeedAt12VoltsMps;
        steerClosedLoopOutput = constants.SteerMotorClosedLoopOutput;
        driveClosedLoopOutput = constants.DriveMotorClosedLoopOutput;
    }

    public SwerveModulePosition getPosition(boolean refreshSignals) {
        double drive_rot = sigDrivePosition;
        double angle_rot = sigSteerPosition;
        drive_rot -= angle_rot * couplingRatioDriveRotorToCANCoder;
        internalState.distanceMeters = drive_rot / driveRotationsPerMeter;
        internalState.angle = Rotation2d.fromRotations(angle_rot);
        return internalState;
    }


    public void apply(SwerveModuleState state, SwerveModule.DriveRequestType driveRequestType) {
        apply(state, driveRequestType, SwerveModule.SteerRequestType.MotionMagic);
    }

    public void apply(SwerveModuleState state, SwerveModule.DriveRequestType driveRequestType, SwerveModule.SteerRequestType steerRequestType) {
        double angleToSetDeg;
        SwerveModuleState optimized;
        optimized = SwerveModuleState.optimize(state, internalState.angle);
        angleToSetDeg = optimized.angle.getRotations();

//        System.out.println(optimized.speedMetersPerSecond);

        // README: why use setters instead of instantiating a new one every call?
        // well first the original implementation used it.
        // second this method gets called quite a lot; if instantiation is used,
        // it may overwhelm the GC and crash the entire robot!

        // steer
        requestSwitch:
        switch (steerRequestType) {
            case MotionMagic:
                switch (steerClosedLoopOutput) {
                    case Voltage:
                        steerSim.setControl(angleVoltageSetter.withPosition(angleToSetDeg));
                        break requestSwitch;
                    case TorqueCurrentFOC:
                        steerSim.setControl(angleTorqueSetter.withPosition(angleToSetDeg));
                    default:
                        break requestSwitch;
                }
            case MotionMagicExpo:
                switch (steerClosedLoopOutput) {
                    case Voltage:
                        steerSim.setControl(angleVoltageExpoSetter.withPosition(angleToSetDeg));
                        break;
                    case TorqueCurrentFOC:
                        steerSim.setControl(angleTorqueExpoSetter.withPosition(angleToSetDeg));
                }
        }

        // drive
        double velocityToSet = optimized.speedMetersPerSecond * driveRotationsPerMeter;
        double steerMotorError = angleToSetDeg - sigSteerPosition;
        double cosineScalar = Math.cos(Units.rotationsToRadians(steerMotorError));
        if (cosineScalar < 0.0) {
            cosineScalar = 0.0;
        }

        velocityToSet *= cosineScalar;
        double azimuthTurnRps = sigSteerVelocity;
        double driveRateBackOut = azimuthTurnRps * couplingRatioDriveRotorToCANCoder;
        velocityToSet += driveRateBackOut;
        switch (driveRequestType) {
            case OpenLoopVoltage:
                velocityToSet /= driveRotationsPerMeter;
                driveSim.setControl(voltageOpenLoopSetter.withOutput(velocityToSet / speedAt12VoltsMps * 12.0));
                break;
            case Velocity:
                switch (driveClosedLoopOutput) {
                    case Voltage:
                        driveSim.setControl(velocityVoltageSetter.withVelocity(velocityToSet));
                        break;
                    case TorqueCurrentFOC:
                        driveSim.setControl(velocityTorqueSetter.withVelocity(velocityToSet));
                }
        }
//        this.module.DriveMotor.setInputVoltage(velocityToSet / speedAt12VoltsMps * 12.0);
//        System.out.println(velocityToSet + " " + angleToSetDeg);
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
                sigDriveVelocity / driveRotationsPerMeter,
                Rotation2d.fromRotations(sigSteerPosition)
        );
    }

    public void update(double dtSeconds, double supplyVoltage) {
        TalonFXSimState steerMotor = steerSim.getSimState();
        TalonFXSimState driveMotor = driveSim.getSimState();
        CANcoderSimState cancoder = CANcoder.getSimState();
        steerMotor.Orientation = this.module.SteerMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        driveMotor.Orientation = this.module.DriveMotorInverted ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
        steerMotor.setSupplyVoltage(supplyVoltage);
        driveMotor.setSupplyVoltage(supplyVoltage);
        cancoder.setSupplyVoltage(supplyVoltage);
        this.module.SteerMotor.setInputVoltage(this.addFriction(steerMotor.getMotorVoltage(), this.module.SteerFrictionVoltage));
        this.module.DriveMotor.setInputVoltage(this.addFriction(driveMotor.getMotorVoltage(), this.module.DriveFrictionVoltage));
        this.module.SteerMotor.update(dtSeconds);
        this.module.DriveMotor.update(dtSeconds);
        steerMotor.setRawRotorPosition(this.module.SteerMotor.getAngularPositionRotations() * this.module.SteerGearing);
        steerMotor.setRotorVelocity(this.module.SteerMotor.getAngularVelocityRPM() / 60.0 * this.module.SteerGearing);
        cancoder.setRawPosition(this.module.SteerMotor.getAngularPositionRotations());
        cancoder.setVelocity(this.module.SteerMotor.getAngularVelocityRPM() / 60.0);
        driveMotor.setRawRotorPosition(this.module.DriveMotor.getAngularPositionRotations() * this.module.DriveGearing);
        driveMotor.setRotorVelocity(this.module.DriveMotor.getAngularVelocityRPM() / 60.0 * this.module.DriveGearing);

        sigDrivePosition = this.module.DriveMotor.getAngularPositionRotations() * this.module.DriveGearing;
        sigDriveVelocity = this.module.DriveMotor.getAngularVelocityRPM() / 60.0 * this.module.DriveGearing;
        sigSteerPosition = this.module.SteerMotor.getAngularPositionRotations() * this.module.SteerGearing;
        sigSteerVelocity = this.module.SteerMotor.getAngularVelocityRPM() / 60.0 * this.module.SteerGearing;

//        double angleChange = this.swerveDriveKinematics.toChassisSpeeds(states).omegaRadiansPerSecond * dtSeconds;
//        this.lastAngle = this.lastAngle.plus(Rotation2d.fromRadians(angleChange));
//        this.pigeonSim.setRawYaw(this.lastAngle.getDegrees());
    }

    protected double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }

        return motorVoltage;
    }

    public static class SimSwerveModule {
        public final DCMotorSim SteerMotor;
        public final DCMotorSim DriveMotor;
        public final double SteerGearing;
        public final double DriveGearing;
        public final double SteerFrictionVoltage;
        public final double DriveFrictionVoltage;
        public final boolean SteerMotorInverted;
        public final boolean DriveMotorInverted;

        public SimSwerveModule(double steerGearing, double steerInertia, double steerFrictionVoltage, boolean steerMotorInverted, double driveGearing, double driveInertia, double driveFrictionVoltage, boolean driveMotorInverted) {
            this.SteerMotor = new DCMotorSim(DCMotor.getKrakenX60Foc(1), steerGearing, steerInertia);
            this.DriveMotor = new DCMotorSim(DCMotor.getKrakenX60Foc(1), driveGearing, driveInertia);
            this.SteerGearing = steerGearing;
            this.DriveGearing = driveGearing;
            this.SteerFrictionVoltage = steerFrictionVoltage;
            this.DriveFrictionVoltage = driveFrictionVoltage;
            this.SteerMotorInverted = steerMotorInverted;
            this.DriveMotorInverted = driveMotorInverted;
        }
    }
}
