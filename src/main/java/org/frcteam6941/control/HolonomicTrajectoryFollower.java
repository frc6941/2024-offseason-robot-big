package org.frcteam6941.control;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class HolonomicTrajectoryFollower extends PathPlannerTrajectoryFollowerBase<HolonomicDriveSignal> {
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;
    private final SimpleMotorFeedforward feedforward;

    private PathPlannerTrajectory.PathPlannerState lastState = null;
    private Pose2d actualPose = null;

    private boolean finished = false;
    private boolean requiredOnTarget = false;
    private boolean lockAngle = true;

    private double TARGET_DISTANCE_ACCURACY_REQUIREMENT = 0.25;
    private double TARGET_VELOCITY_ACCURACY_REQUIREMENT = 0.25;

    public HolonomicTrajectoryFollower(PIDController xController, PIDController yController,
            ProfiledPIDController thetaController, SimpleMotorFeedforward feedforward) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        this.feedforward = feedforward;

        this.xController.setTolerance(TARGET_DISTANCE_ACCURACY_REQUIREMENT, TARGET_VELOCITY_ACCURACY_REQUIREMENT);
        this.yController.setTolerance(TARGET_DISTANCE_ACCURACY_REQUIREMENT, TARGET_VELOCITY_ACCURACY_REQUIREMENT);
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(Pose2d currentPose, Translation2d velocity,
            double rotationalVelocity, PathPlannerTrajectory trajectory, double time,
            double dt) {
        if (time > trajectory.getTotalTimeSeconds()) {
            if (this.requiredOnTarget) {
                if (this.xController.atSetpoint() && this.yController.atSetpoint()) {
                    finished = true;
                    return new HolonomicDriveSignal(new Translation2d(0, 0), 0.0, true, false);
                }
            } else {
                finished = true;
                return new HolonomicDriveSignal(new Translation2d(0, 0), 0.0, true, false);
            }
        }

        actualPose = currentPose;

        lastState = (PathPlannerState) trajectory.sample(time);
        double x = xController.calculate(currentPose.getX(), lastState.poseMeters.getX());
        double y = yController.calculate(currentPose.getY(), lastState.poseMeters.getY());
        double rotation = 0.0;
        Translation2d translationVector = new Translation2d(x, y);

        
        if (this.lastState != null) {
            Translation2d targetDisplacement = lastState.poseMeters.getTranslation()
                    .minus(this.lastState.poseMeters.getTranslation());
            double feedForwardGain = feedforward.calculate(lastState.velocityMetersPerSecond,
                    lastState.accelerationMetersPerSecondSq) / 12.0;

            if (targetDisplacement.getNorm() != 0.00) { // Prevent NaN cases
                Translation2d feedForwardVector = targetDisplacement.times(feedForwardGain / targetDisplacement.getNorm());
                translationVector = translationVector.plus(feedForwardVector);
            }
        }

        if (this.lockAngle) {
            rotation = this.thetaController.calculate(currentPose.getRotation().getDegrees(), lastState.holonomicRotation.getDegrees());
        }

        return new HolonomicDriveSignal(
                translationVector,
                rotation,
                true,
                false
        );
    }

    public PathPlannerTrajectory.PathPlannerState getLastState() {
        return lastState;
    }

    public void setLockAngle(boolean lock) {
        this.lockAngle = lock;
    }

    public void setRequiredOnTarget(boolean requiredOnTarget) {
        this.requiredOnTarget = requiredOnTarget;
    }

    public void setTolerance(double distance, double velocity) {
        this.TARGET_DISTANCE_ACCURACY_REQUIREMENT = distance;
        this.TARGET_VELOCITY_ACCURACY_REQUIREMENT = velocity;
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    public boolean isPathFollowing() {
        return !finished;
    }

    public void sendData() {
        if (isPathFollowing()) {
            PathPlannerServer.sendActivePath(getCurrentTrajectory().get().getStates());
            PathPlannerServer.sendPathFollowingData(new Pose2d(lastState.poseMeters.getTranslation(), lastState.holonomicRotation), actualPose);
        }
    }

    @Override
    protected void reset() {
        this.xController.reset();
        this.yController.reset();
        this.finished = false;
    }
}
