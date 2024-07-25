package org.frcteam6941.swerve;

import org.frcteam1678.lib.util.CTREModuleState;
import org.frcteam6941.utils.AngleNormalization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * SJTU Swerve Module Mark 5.
 * This is a basic implementation of {@link SwerveModuleBase}.
 */
public class SJTUMK5iModuleSim implements SwerveModuleBase {
    private SwerveDrivetrainConstants drivetrainConstants;
    private SwerveModuleConstants moduleConstants;

    private double driveVelocity = 0.0;
    private double anglePosition = 0.0;

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
    public SJTUMK5iModuleSim(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants moduleConstants) {
        this.drivetrainConstants = drivetrainConstants;
        this.moduleConstants = moduleConstants;
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
            driveVelocity = optimizedState.speedMetersPerSecond / 1.0 * drivetrainConstants.getFreeSpeedMetersPerSecond();
        } else {
            driveVelocity = optimizedState.speedMetersPerSecond;
        }

        anglePosition = desiredState.angle.getDegrees();
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
        return Rotation2d.fromDegrees(anglePosition);
    }

    /**
     * Get the state of the module.
     * 
     * @return The state of the module.
     */
    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveVelocity, getEncoder());
    }

    @Override
    public int getModuleNumber() {
        return moduleConstants.getModuleNumber();
    }

    @Override
    public double getTick() {
        return 0.0;
    }
}
