package org.frcteam6941.localization;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import lombok.Synchronized;
import org.frcteam6941.utils.InterpolatingTreeMap;
import org.frcteam6941.utils.MovingAveragePose2d;

public class SwerveDeltaCoarseLocalizer implements Localizer {
    private final Object statusLock = new Object();
    private SwerveDriveOdometry swerveOdometry;
    private SwerveDrivePoseEstimator poseEstimator;
    private Pose2d previousPose = null;
    private Pose2d previousVelocity = new Pose2d();
    private double distanceDriven = 0.0;
    private InterpolatingTreeMap<Double, Pose2d> fieldToVehicle;
    private int poseBufferSize;
    private int velocityBufferSize;
    private int accelerationBufferSize;
    private Pose2d vehicleVelocityMeasured;
    private MovingAveragePose2d vehicleVelocityMeasuredFilter;
    private Pose2d vehicleAccelerationMeasured;
    private MovingAveragePose2d vehicleAccelerationMeasuredFilter;
    private Pose2d vehicleVelocityPredicted;
    private MovingAveragePose2d vehicleVelocityPredictedFilter;

    public SwerveDeltaCoarseLocalizer(SwerveDriveKinematics kinematics, int poseBufferSize, int velocityBufferSize,
                                      int accelerationBufferSize, SwerveModulePosition[] initPosition) {
        this.poseBufferSize = poseBufferSize;
        this.velocityBufferSize = velocityBufferSize;
        this.accelerationBufferSize = accelerationBufferSize;
        fieldToVehicle = new InterpolatingTreeMap<Double, Pose2d>(poseBufferSize);
        vehicleVelocityMeasured = new Pose2d();
        vehicleVelocityMeasuredFilter = new MovingAveragePose2d(velocityBufferSize);
        vehicleAccelerationMeasured = new Pose2d();
        vehicleAccelerationMeasuredFilter = new MovingAveragePose2d(accelerationBufferSize);
        vehicleVelocityPredicted = new Pose2d();
        vehicleVelocityPredictedFilter = new MovingAveragePose2d(velocityBufferSize);

        vehicleVelocityMeasuredFilter.add(new Pose2d());
        vehicleAccelerationMeasuredFilter.add(new Pose2d());

        swerveOdometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), initPosition);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, new Rotation2d(), initPosition, new Pose2d());
    }

    public synchronized Pose2d updateWithTime(double time, double dt, Rotation2d gyroAngle,
                                              SwerveModulePosition[] moduleStates) {
        dt = Constants.LOOPER_DT;
        synchronized (statusLock) {
            // Get pose from kinematics update
            Pose2d pose = swerveOdometry.update(gyroAngle, moduleStates);
            poseEstimator.updateWithTime(time, gyroAngle, moduleStates);

            // First, get the displacement
            if (previousPose == null) {
                previousPose = pose;
            }
            Pose2d poseDelta = pose.relativeTo(previousPose);

            distanceDriven += poseDelta.getTranslation().getNorm();
            fieldToVehicle.put(time, new Pose2d(
                    fieldToVehicle.lastEntry().getValue().getTranslation().plus(poseDelta.getTranslation()),
                    fieldToVehicle.lastEntry().getValue().getRotation().rotateBy(poseDelta.getRotation())));

            vehicleVelocityMeasured = new Pose2d(poseDelta.getX() / dt, poseDelta.getY() / dt,
                    poseDelta.getRotation().times(1.0 / dt));
            vehicleVelocityMeasuredFilter.add(vehicleVelocityMeasured);


            // Second, get the acceleration
            if (previousVelocity == null) {
                previousVelocity = vehicleVelocityMeasured;
            }

            Pose2d velocityDelta = vehicleVelocityMeasured.relativeTo(previousVelocity);
            vehicleAccelerationMeasured = new Pose2d(velocityDelta.getX() / dt, velocityDelta.getY() / dt,
                    velocityDelta.getRotation().times(1.0 / dt));
            vehicleAccelerationMeasuredFilter.add(vehicleAccelerationMeasured);

            // Third, update prediction using acceleration
            vehicleVelocityPredicted = vehicleVelocityMeasuredFilter.getAverage()
                    .plus(new Transform2d(vehicleAccelerationMeasured.getTranslation(),
                            vehicleAccelerationMeasured.getRotation()).times(dt));
            vehicleVelocityPredictedFilter.add(vehicleVelocityPredicted);

            // Finally, update system state and ready for the next iteration
            previousPose = pose;
            previousVelocity = vehicleVelocityMeasured;


            // Return pose
            return pose;
        }
    }

    @Override
    @Synchronized
    public synchronized Pose2d getLatestPose() {
        synchronized (statusLock) {
            return swerveOdometry.getPoseMeters();
        }
    }

    @Override
    @Synchronized
    public Pose2d getCoarseFieldPose(double time) {
        synchronized (statusLock) {
            return poseEstimator.getEstimatedPosition();
        }
    }

    @Override
    @Synchronized
    public synchronized Pose2d getMeasuredVelocity() {
        synchronized (statusLock) {
            return vehicleVelocityMeasured;
        }
    }

    @Override
    @Synchronized
    public synchronized Pose2d getPredictedVelocity() {
        synchronized (statusLock) {
            return vehicleVelocityPredicted;
        }
    }

    @Override
    @Synchronized
    public synchronized Pose2d getSmoothedPredictedVelocity() {
        synchronized (statusLock) {
            return vehicleVelocityPredictedFilter.getAverage();
        }
    }

    @Override
    @Synchronized
    public synchronized Pose2d getPredictedPose(double lookahead) {
        synchronized (statusLock) {
            return getLatestPose().transformBy(new Transform2d(getSmoothedPredictedVelocity().getTranslation(),
                    getSmoothedPredictedVelocity().getRotation()).times(-lookahead));
        }
    }

    @Override
    @Synchronized
    public synchronized Pose2d getMeasuredAcceleration() {
        synchronized (statusLock) {
            return vehicleAccelerationMeasured;
        }
    }

    @Override
    @Synchronized
    public synchronized Pose2d getSmoothedVelocity() {
        synchronized (statusLock) {
            return vehicleVelocityMeasuredFilter.getAverage();
        }
    }

    @Override
    public synchronized Pose2d getSmoothedAccleration() {
        synchronized (statusLock) {
            return vehicleAccelerationMeasuredFilter.getAverage();
        }
    }

    @Override
    public synchronized Pose2d getPoseAtTime(double time) {
        synchronized (statusLock) {
            return fieldToVehicle.getInterpolated(time, 0.5);
        }
    }

    @Override
    public synchronized double getDistanceDriven() {
        return distanceDriven;
    }

    @Override
    public synchronized void addMeasurement(double time, Pose2d measuredPose, Pose2d stdDeviation) {
        synchronized (statusLock) {
            poseEstimator.addVisionMeasurement(
                    measuredPose,
                    time,
                    MatBuilder.fill(Nat.N3(), Nat.N1(), stdDeviation.getX(), stdDeviation.getY(), stdDeviation.getRotation().getDegrees())
            );
        }
    }

    public synchronized void reset(Pose2d resetPose, SwerveModulePosition[] modulePositions) {
        synchronized (statusLock) {
            swerveOdometry.resetPosition(resetPose.getRotation(), modulePositions, resetPose);
            poseEstimator.resetPosition(resetPose.getRotation(), modulePositions, resetPose);
            previousPose = null;
            fieldToVehicle = new InterpolatingTreeMap<Double, Pose2d>(poseBufferSize);
            fieldToVehicle.put(Timer.getFPGATimestamp(), resetPose);
            vehicleVelocityMeasured = new Pose2d();
            vehicleVelocityMeasuredFilter = new MovingAveragePose2d(velocityBufferSize);
            vehicleAccelerationMeasured = new Pose2d();
            vehicleAccelerationMeasuredFilter = new MovingAveragePose2d(accelerationBufferSize);
            vehicleVelocityPredictedFilter = new MovingAveragePose2d(velocityBufferSize);

            vehicleVelocityMeasuredFilter.add(new Pose2d());
            vehicleAccelerationMeasuredFilter.add(new Pose2d());
        }
    }
}
