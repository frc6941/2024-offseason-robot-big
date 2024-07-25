package org.frcteam6941.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.frcteam1678.lib.math.Conversions;
import org.frcteam1678.lib.util.CTREModuleState;
import org.frcteam6941.utils.AngleNormalization;
import org.frcteam6941.utils.CTREFactory;
import org.frcteam6941.utils.Conversions4096;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * SJTU Swerve Module Mark 5.
 * This is a basic implementation of {@link SwerveModuleBase}.
 */
public class SJTUMK5iModule implements SwerveModuleBase {
    private Double recordAngle = null;
    private SwerveDrivetrainConstants drivetrainConstants;
    private SwerveModuleConstants moduleConstants;

    private TalonSRX angleMotor;
    private TalonFX driveMotor;

    /**
     * Constructor function for the module.
     * 
     * @param moduleNumber The number of the module. This will be correspondent to
     *                     the position of its {@link Translation2d}
     *                     in {@link SwerveDriveKinematics}.
     * @param driveMotorID CAN ID of the falcon drive motor.
     * @param angleMotorID CAN ID of the 775pro turning motor along with the
     *                     TalonSRX motor controller.
     * @param angleOffset  Angle offset for the encoder in degrees.
     */
    public SJTUMK5iModule(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants moduleConstants) {
        this.drivetrainConstants = drivetrainConstants;
        this.moduleConstants = moduleConstants;
        /* Angle Motor Config */
        angleMotor = CTREFactory.createDefaultTalonSRX(moduleConstants.getAngleMotorPort());
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = CTREFactory.createDefaultTalonFX(moduleConstants.getDriveMotorPort(), moduleConstants.isOnCanivore());
        configDriveMotor();
    }

    /**
     * Core function to set the state of the swerve module.
     * 
     * @param desiredState   The desired state of the module.
     * @param isOpenLoop     Whether the speed control will be open loop (voltage
     *                       control), or close loop (using on-board PIDF control to
     *                       reach the velocity set point).
     * @param overrideMotion Enable angle control even if the speed is lower than
     *                       the limit. Usually used for BRAKE mode settings.
     */
    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean overrideMotion) {
        SwerveModuleState optimizedState = CTREModuleState.optimize(
                desiredState,
                Rotation2d.fromDegrees(this.getEncoderUnbound().getDegrees()));
        if (isOpenLoop) {
            driveMotor.set(ControlMode.PercentOutput, optimizedState.speedMetersPerSecond);
        } else {
            double velocity = Conversions.MPSToFalcon(
                optimizedState.speedMetersPerSecond,
                drivetrainConstants.getWheelCircumferenceMeters(),
                drivetrainConstants.getDriveGearRatio()
            );
            driveMotor.set(ControlMode.Velocity, velocity);
        }

        boolean inMotion; // Preventing jittering and useless resetting.
        if (isOpenLoop) {
            inMotion = Math.abs(optimizedState.speedMetersPerSecond) >= drivetrainConstants.getDeadband();
        } else {
            inMotion = Math.abs(optimizedState.speedMetersPerSecond) >= (drivetrainConstants.getFreeSpeedMetersPerSecond() * drivetrainConstants.getDeadband());
        }

        if (inMotion || overrideMotion) {
            double target = optimizedState.angle.getDegrees();
            angleMotor.set(ControlMode.Position, (target + moduleConstants.getAngleOffsetDegreesCCW()) / 360.0 * 4096.0);
            recordAngle = null;
        } else {
            if (recordAngle == null) {
                recordAngle = (this.getEncoderUnbound().getDegrees() + moduleConstants.getAngleOffsetDegreesCCW()) / 360.0 * 4096.0;
            }
            angleMotor.set(ControlMode.Position, recordAngle);
        }
    }

    /** Configurations for the angle motor. */
    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        angleMotor.setInverted(moduleConstants.isInvertAngleOutput());
        angleMotor.setSensorPhase(moduleConstants.isInvertAngleSensorPhase());
        angleMotor.setNeutralMode(NeutralMode.Brake);
        angleMotor.configNeutralDeadband(0.01);

        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 30, 30, 0.01);
        angleMotor.configSupplyCurrentLimit(curr_lim);

        angleMotor.config_kP(0, drivetrainConstants.getAngleKP());
        angleMotor.config_kI(0, drivetrainConstants.getAngleKI());
        angleMotor.config_kD(0, drivetrainConstants.getAngleKD());
        angleMotor.config_IntegralZone(0, 100.0);
        angleMotor.configVoltageCompSaturation(12);
        angleMotor.enableVoltageCompensation(true);
    }

    /** Configurations for the drive motor. */
    private void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        driveMotor.setInverted(true);
        driveMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.configNeutralDeadband(0.005);

        SupplyCurrentLimitConfiguration curr_lim = new SupplyCurrentLimitConfiguration(true, 30, 40, 0.02);
        driveMotor.configSupplyCurrentLimit(curr_lim);

        driveMotor.config_kP(0, drivetrainConstants.getDriveKP());
        driveMotor.config_kI(0, drivetrainConstants.getDriveKI());
        driveMotor.config_kD(0, drivetrainConstants.getDriveKD());
        driveMotor.config_kF(0, drivetrainConstants.getDriveKV());
        driveMotor.config_IntegralZone(0, 100.0);
        driveMotor.configVoltageCompSaturation(12);
        driveMotor.enableVoltageCompensation(true);
    }

    /**
     * Get the Encoder angle within 0 to 360 degrees.
     * 
     * @return The normalized angle of the encoder in degrees.
     */
    private Rotation2d getEncoder() {
        return Rotation2d.fromDegrees(AngleNormalization.getAbsoluteAngleDegree(getEncoderUnbound().getDegrees()));
    }

    /**
     * Get the Encoder angle unbound (maybe greater than 360 or lower than 0) with
     * angle offset calculated.
     * 
     * @return The raw angle of the encoder in degrees.
     */
    private Rotation2d getEncoderUnbound() {
        return Rotation2d.fromDegrees(
                Conversions4096.falconToDegrees(angleMotor.getSelectedSensorPosition(), 1.0) - moduleConstants.getAngleOffsetDegreesCCW());
    }

    /**
     * Get the state of the module.
     * 
     * @return The state of the module.
     */
    @Override
    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), drivetrainConstants.getWheelCircumferenceMeters(), drivetrainConstants.getDriveGearRatio());
        Rotation2d angle = getEncoder();
        return new SwerveModuleState(velocity, angle);
    }

    @Override
    public int getModuleNumber() {
        return moduleConstants.getModuleNumber();
    }

    @Override
    public double getTick() {
        return driveMotor.getSelectedSensorPosition();
    }
}
