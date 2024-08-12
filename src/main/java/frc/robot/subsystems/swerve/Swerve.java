package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team254.lib.util.MovingAverage;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.utils.Utils;
import lombok.Getter;
import lombok.Setter;
import lombok.Synchronized;
import org.frcteam6941.control.HolonomicDriveSignal;
import org.frcteam6941.drivers.DummyGyro;
import org.frcteam6941.drivers.Gyro;
import org.frcteam6941.drivers.Pigeon2Gyro;
import org.frcteam6941.localization.Localizer;
import org.frcteam6941.localization.SwerveDeltaCoarseLocalizer;
import org.frcteam6941.looper.Updatable;
import org.frcteam6941.swerve.*;
import org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.frcteam6941.utils.AngleNormalization;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.SwerveConstants.speedAt12Volts;

/**
 * Rectangular Swerve Drivetrain.
 */
public class Swerve implements Updatable, Subsystem {
    private static Swerve instance;
    private final SwerveModuleBase[] swerveMods;
    private final SwerveDriveKinematics swerveKinematics;
    private final SwerveDeltaCoarseLocalizer swerveLocalizer;
    @Getter
    private final Gyro gyro;
    private final SwerveSetpointGenerator generator;
    // System Status
    private final MovingAverage pitchVelocity;
    private final MovingAverage rollVelocity;
    private final MovingAverage yawVelocity;
    // Logging
    private final NetworkTable dataTable = NetworkTableInstance.getDefault().getTable("Swerve");
    // Snap Rotation Controller
    private final ProfiledPIDController headingController;
    private boolean isLockHeading;
    /**
     * -- GETTER --
     * Get the lock heading target for the swerve drive.
     */
    @Getter
    private double headingTarget = 0.0;
    @Getter
    @Setter
    private Double overrideRotation = null;
    @Getter
    @Setter
    private double headingVelocityFeedforward = 0.00;
    // Control Targets
    private HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(new Translation2d(), 0.0, true, false);
    private HolonomicDriveSignal autoDriveSignal = new HolonomicDriveSignal(new Translation2d(), 0.0, true, false);

    private SwerveSetpoint setpoint;
    private SwerveSetpoint previousSetpoint;
    @Getter
    private KinematicLimits kinematicLimits;
    @Getter
    @Setter
    private State state = State.DRIVE;

    private Swerve() {
        if (RobotBase.isReal()) {
            swerveMods = new SwerveModuleBase[]{
                    new CTRESwerveModule(0, Constants.SwerveConstants.FrontLeft,
                            Constants.RobotConstants.CAN_BUS_NAME),
                    new CTRESwerveModule(1, Constants.SwerveConstants.FrontRight,
                            Constants.RobotConstants.CAN_BUS_NAME),
                    new CTRESwerveModule(2, Constants.SwerveConstants.BackLeft, Constants.RobotConstants.CAN_BUS_NAME),
                    new CTRESwerveModule(3, Constants.SwerveConstants.BackRight,
                            Constants.RobotConstants.CAN_BUS_NAME),
            };
            gyro = new Pigeon2Gyro(Constants.SwerveConstants.PIGEON_ID, Constants.RobotConstants.CAN_BUS_NAME);
        } else {
            swerveMods = new SwerveModuleBase[]{
                    new SimSwerveModuleDummy(0, Constants.SwerveConstants.FrontLeft),
                    new SimSwerveModuleDummy(1, Constants.SwerveConstants.FrontRight),
                    new SimSwerveModuleDummy(2, Constants.SwerveConstants.BackLeft),
                    new SimSwerveModuleDummy(3, Constants.SwerveConstants.BackRight),
            };
            gyro = new DummyGyro(Constants.LOOPER_DT);
        }

        swerveKinematics = new SwerveDriveKinematics(
                Constants.SwerveConstants.modulePlacements);
        swerveLocalizer = new SwerveDeltaCoarseLocalizer(swerveKinematics, 50, 20, 20, getModulePositions());

        gyro.setYaw(0.0);
        swerveLocalizer.reset(new Pose2d(), getModulePositions());

        yawVelocity = new MovingAverage(10);
        pitchVelocity = new MovingAverage(10);
        rollVelocity = new MovingAverage(10);

        setpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());
        previousSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates());
        generator = new SwerveSetpointGenerator(Constants.SwerveConstants.modulePlacements);
        kinematicLimits = Constants.SwerveConstants.DRIVETRAIN_UNCAPPED;

        headingController = new ProfiledPIDController(
                Constants.SwerveConstants.headingController.HEADING_KP.get(),
                Constants.SwerveConstants.headingController.HEADING_KI.get(),
                Constants.SwerveConstants.headingController.HEADING_KD.get(),
                new TrapezoidProfile.Constraints(6000, 7200));
        headingController.setIntegratorRange(-0.5, 0.5);
        headingController.enableContinuousInput(0, 360.0);

        var driveBaseRadius = getDriveBaseRadius();

        AutoBuilder.configureHolonomic(
                () -> swerveLocalizer.getCoarseFieldPose(0),
                this::resetPose,
                this::getChassisSpeeds,
                this::driveSpeed,
                new HolonomicPathFollowerConfig(
                        speedAt12Volts.magnitude(),
                        driveBaseRadius,
                        new ReplanningConfig()),
                Utils::flip,
                this
        );
    }

    private static double getDriveBaseRadius() {
        var moduleLocations = new Translation2d[]{
                new Translation2d(Constants.SwerveConstants.FrontLeft.LocationX,
                        Constants.SwerveConstants.FrontLeft.LocationY),
                new Translation2d(Constants.SwerveConstants.FrontRight.LocationX,
                        Constants.SwerveConstants.FrontRight.LocationY),
                new Translation2d(Constants.SwerveConstants.BackLeft.LocationX,
                        Constants.SwerveConstants.BackLeft.LocationY),
                new Translation2d(Constants.SwerveConstants.BackRight.LocationX,
                        Constants.SwerveConstants.BackRight.LocationY)
        };

        var driveBaseRadius = 0.0;
        for (var moduleLocation : moduleLocations)
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        return driveBaseRadius;
    }

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private void driveSpeed(ChassisSpeeds speeds) {
        autoDrive(new Translation2d(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond
        ), speeds.omegaRadiansPerSecond, true, false);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Core methods to update the odometry of swerve based on module states.
     *
     * @param time Current time stamp.
     * @param dt   Delta time between updates.
     */
    private void updateOdometry(double time, double dt) {
        swerveLocalizer.updateWithTime(time, dt, gyro.getYaw(), getModulePositions());
    }


    /**
     * Core method to update swerve modules according to the
     * {@link HolonomicDriveSignal} given.
     *
     * @param driveSignal The holonomic drive signal.
     * @param dt          Delta time between updates.
     */
    private void updateModules(HolonomicDriveSignal driveSignal, double dt) {
        ChassisSpeeds desiredChassisSpeed;

        if (driveSignal == null) {
            desiredChassisSpeed = new ChassisSpeeds(0.0, 0.0, 0.0);
            driveSignal = new HolonomicDriveSignal(new Translation2d(), 0.0, true, false);
        } else {
            double x = driveSignal.getTranslation().getX();
            double y = driveSignal.getTranslation().getY();
            double rotation = driveSignal.getRotation();
            Rotation2d robotAngle = swerveLocalizer.getLatestPose().getRotation();

            if (driveSignal.isFieldOriented())
                desiredChassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, robotAngle);
            else
                desiredChassisSpeed = new ChassisSpeeds(x, y, rotation);
        }

        Twist2d twist = new Pose2d().log(new Pose2d(
                new Translation2d(
                        desiredChassisSpeed.vxMetersPerSecond * Constants.LOOPER_DT,
                        desiredChassisSpeed.vyMetersPerSecond * Constants.LOOPER_DT),
                new Rotation2d(
                        desiredChassisSpeed.omegaRadiansPerSecond * Constants.LOOPER_DT)));

        desiredChassisSpeed = new ChassisSpeeds(
                twist.dx / Constants.LOOPER_DT,
                twist.dy / Constants.LOOPER_DT,
                twist.dtheta / Constants.LOOPER_DT);

        setpoint = generator.generateSetpoint(
                kinematicLimits, previousSetpoint, desiredChassisSpeed, dt);
        previousSetpoint = setpoint;

        for (SwerveModuleBase mod : swerveMods) {
            mod.setDesiredState(setpoint.mModuleStates[mod.getModuleNumber()], driveSignal.isOpenLoop(), false);
        }
    }

    @Synchronized
    public double getYawVelocity() {
        return yawVelocity.getAverage();
    }

    @Synchronized
    public double getPitchVelocity() {
        return pitchVelocity.getAverage();
    }

    @Synchronized
    public double getRollVelocity() {
        return rollVelocity.getAverage();
    }

    /**
     * Core method to drive the swerve drive. Note that any trajectory following
     * signal will be canceled when this method is called.
     *
     * @param translationalVelocity Normalized translation vector of the swerve
     *                              drive.
     * @param rotationalVelocity    Normalized rotational magnitude of the swerve
     *                              drive.
     * @param isFieldOriented       Is the drive signal field oriented.
     */
    public void drive(Translation2d translationalVelocity, double rotationalVelocity,
                      boolean isFieldOriented, boolean isOpenLoop) {
        // if (Math.hypot(translationalVelocity.getX(), translationalVelocity.getY())
        // < Constants.SwerveConstants.deadband) {
        // translationalVelocity = new Translation2d(0, 0);
        // }
        if (Math.abs(translationalVelocity.getX()) < Constants.SwerveConstants.deadband) {
            translationalVelocity = new Translation2d(0, translationalVelocity.getY());
        }
        if (Math.abs(translationalVelocity.getY()) < Constants.SwerveConstants.deadband) {
            translationalVelocity = new Translation2d(translationalVelocity.getX(), 0);
        }
        if (Math.abs(rotationalVelocity) < Constants.SwerveConstants.rotationalDeadband) {
            rotationalVelocity = 0;
        }
        driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented, isOpenLoop);
    }

    public void autoDrive(Translation2d translationalVelocity, double rotationalVelocity,
                          boolean isFieldOriented, boolean isOpenLoop) {
        autoDriveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, isFieldOriented, isOpenLoop);
    }

    public void pointWheelsAt(Rotation2d rotation2d) {
        for (SwerveModuleBase mod : swerveMods) {
            // System.out.println(setpoint.mModuleStates[mod.getModuleNumber()]);//add
            setpoint.mModuleStates[mod.getModuleNumber()].angle = rotation2d;
            setpoint.mModuleStates[mod.getModuleNumber()].speedMetersPerSecond = 0.0;
            mod.setDesiredState(setpoint.mModuleStates[mod.getModuleNumber()], true, false);
        }
    }

    public void brake() {
        setState(State.BRAKE);
    }

    public void normal() {
        setState(State.DRIVE);
    }

    public void empty() {
        setState(State.EMPTY);
    }

    public void auto() {
        setState(State.PATH_FOLLOWING);
    }

    public void stopMovement() {
        driveSignal = new HolonomicDriveSignal(new Translation2d(), 0.0, true, false);
    }

    public void setKinematicsLimit(KinematicLimits limit) {
        kinematicLimits = limit;
    }

    public void resetPose(Pose2d resetPose) {
        gyro.setYaw(resetPose.getRotation().getDegrees());
        // System.out.println(resetPose.getRotation().getDegrees());
        // System.out.println(gyro.getYaw().getDegrees());
        swerveLocalizer.reset(resetPose, getModulePositions());
    }

    /**
     * Set the state of the module independently.
     *
     * @param desiredStates The states of the model.
     * @param isOpenLoop    Whether to use open loop control
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop, boolean overrideMotion) {
        if (isOpenLoop) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                    Constants.SwerveConstants.MAX_VOLTAGE.magnitude());
        }

        for (SwerveModuleBase mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], isOpenLoop, overrideMotion);
        }
    }

    /**
     * Convenience method to set the wheels in X shape to resist impacts.
     */
    private void setModuleStatesBrake() {
        for (SwerveModuleBase mod : swerveMods) {
            Translation2d modulePosition = Constants.SwerveConstants.modulePlacements[mod.getModuleNumber()];
            Rotation2d angle = new Rotation2d(modulePosition.getX(), modulePosition.getY());
            mod.setDesiredState(new SwerveModuleState(0.0, angle.plus(Rotation2d.fromDegrees(180.0))), false, true);
        }
    }

    public void resetYaw(double degree) {
        this.gyro.setYaw(degree);
    }

    public void resetRoll(double degree) {
        this.gyro.setRoll(degree);
    }

    public void resetPitch(double degree) {
        this.gyro.setPitch(degree);
    }

    public Localizer getLocalizer() {
        return this.swerveLocalizer;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveMods.length];
        for (SwerveModuleBase mod : swerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveMods.length];
        for (SwerveModuleBase mod : swerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void setHeadingControllerPID() {
        headingController.setPID(
                Constants.SwerveConstants.headingController.HEADING_KP.get(),
                Constants.SwerveConstants.headingController.HEADING_KI.get(),
                Constants.SwerveConstants.headingController.HEADING_KD.get());
    }

    /*
     * Reset heading controller according to current drivetrain status.
     */
    public void resetHeadingController() {
        headingController.reset(
                swerveLocalizer.getLatestPose().getRotation().getDegrees(),
                getYawVelocity());
    }

    /**
     * Return if the swerve drive has a set heading target.
     *
     * @return If swerve is in lock heading.
     */
    public boolean isLockHeading() {
        return this.isLockHeading;
    }

    /**
     * Set if swerve will enter lock heading.
     *
     * @param status Boolean value for enabling or disabling lock heading.
     */
    public void setLockHeading(boolean status) {
        if (this.isLockHeading != status) {
            headingController.reset(gyro.getYaw().getDegrees(), getYawVelocity());
        }
        this.isLockHeading = status;
        this.headingVelocityFeedforward = 0.0;
    }

    public synchronized void setHeadingTarget(double heading) {
        double target = heading;
        double position = gyro.getYaw().getDegrees();

        while (position - target > 180) {
            target += 360;
        }

        while (target - position > 180) {
            target -= 360;
        }

        headingTarget = target;
    }

    public boolean isHeadingOnTarget() {
        return this.headingController.atSetpoint();
    }

    public void clearOverrideRotation() {
        overrideRotation = null;
    }

    @Override
    public void read(double time, double dt) {
        for (SwerveModuleBase mod : swerveMods) {
            mod.updateSignals();
        }
        updateOdometry(time, dt);
        SmartDashboard.putString("swerve/localizer/fused_pose", getLocalizer().getCoarseFieldPose(time).toString());
    }

    @Override
    public void update(double time, double dt) {
        if (isLockHeading) {
            headingTarget = AngleNormalization.placeInAppropriate0To360Scope(gyro.getYaw().getDegrees(), headingTarget);
            double rotation = headingController.calculate(gyro.getYaw().getDegrees(), new TrapezoidProfile.State(
                    headingTarget, headingVelocityFeedforward));
            if (this.state == State.PATH_FOLLOWING) {
                autoDriveSignal = new HolonomicDriveSignal(autoDriveSignal.getTranslation(), rotation,
                        autoDriveSignal.isFieldOriented(), autoDriveSignal.isOpenLoop());
                System.out.println(autoDriveSignal);
                //FIXME: path following
            } else {
                driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation(), rotation,
                        driveSignal.isFieldOriented(), driveSignal.isOpenLoop());
            }
            Logger.recordOutput("heading/rotation", rotation);
            Logger.recordOutput("heading/gyro", gyro.getYaw().getDegrees());
            Logger.recordOutput("heading/target", headingTarget);
            Logger.recordOutput("heading/difference", Math.abs(headingTarget - gyro.getYaw().getDegrees()));

        } else if (overrideRotation != null) {
            driveSignal = new HolonomicDriveSignal(driveSignal.getTranslation(), overrideRotation,
                    driveSignal.isFieldOriented(), driveSignal.isOpenLoop());
        }

        rollVelocity.addNumber(gyro.getRaw()[0]);
        pitchVelocity.addNumber(gyro.getRaw()[1]);
        yawVelocity.addNumber(gyro.getRaw()[2]);
    }

    @Override
    public void write(double time, double dt) {
        switch (state) {
            case BRAKE:
                setModuleStatesBrake();
                break;
            case DRIVE:
                updateModules(driveSignal, dt);
                break;
            case PATH_FOLLOWING:
                updateModules(autoDriveSignal, dt);
                break;
            case EMPTY:
                break;
        }
    }

    @Override
    public void telemetry() {
        Pose2d latestPose = swerveLocalizer.getLatestPose();
        dataTable.getEntry("Pose").setDoubleArray(
                new double[]{
                        latestPose.getX(), latestPose.getY(), latestPose.getRotation().getDegrees()
                });
        for (SwerveModuleBase mod : swerveMods) {
            SmartDashboard.putNumber("Swerve/SPEED REQ" + mod.getModuleNumber(),
                    setpoint.mModuleStates[mod.getModuleNumber()].speedMetersPerSecond);
            SmartDashboard.putNumber("Swerve/SPEED ACT" + mod.getModuleNumber(),
                    mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Swerve/ANGLE REQ" + mod.getModuleNumber(),
                    setpoint.mModuleStates[mod.getModuleNumber()].angle.getDegrees());
            SmartDashboard.putNumber("Swerve/ANGLE ACT" + mod.getModuleNumber(),
                    mod.getState().angle.getDegrees());
        }
        SmartDashboard.putNumber("Swerve/headingTarget", headingTarget);
        if (Constants.TUNING) {
            setHeadingControllerPID();
            SmartDashboard.putString("swerve/localizer/latest_pose", getLocalizer().getLatestPose().toString());
            SmartDashboard.putString("swerve/localizer/accel", getLocalizer().getMeasuredAcceleration().toString());
            SmartDashboard.putString("swerve/localizer/velocity", getLocalizer().getSmoothedVelocity().toString());
        }
    }

    @Override
    public void stop() {
        stopMovement();
        setState(State.DRIVE);
    }

    @Override
    public void simulate(double time, double dt) {
        gyro.setYaw(
                gyro.getYaw().rotateBy(
                        new Rotation2d(dt * setpoint.mChassisSpeeds.omegaRadiansPerSecond)
                ).getDegrees()
        );
        read(time, dt);
    }

    public boolean aimingReady(double offset) {
        SmartDashboard.putBoolean("SwerveReady", Math.abs(gyro.getYaw().getDegrees() - headingTarget) < offset);
        return Math.abs(gyro.getYaw().getDegrees() - headingTarget) < offset;
    }

    public enum State {
        BRAKE, DRIVE, PATH_FOLLOWING, EMPTY
    }

}
