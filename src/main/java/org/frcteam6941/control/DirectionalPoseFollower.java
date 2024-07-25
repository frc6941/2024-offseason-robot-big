package org.frcteam6941.control;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DirectionalPoseFollower {
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;

    private DirectionalPose2d targetPose = null;

    public DirectionalPoseFollower(ProfiledPIDController xController, ProfiledPIDController yController) {
        this.xController = xController;
        this.yController = yController;
    }

    public void setTargetPose(DirectionalPose2d targetPose, Pose2d currentPosition, Pose2d currentVelocity) {
        if(this.targetPose == null) {
            xController.reset(currentPosition.getX(), currentVelocity.getX());
            yController.reset(currentPosition.getY(), currentVelocity.getY());
        }
        this.targetPose = targetPose;
    }

    public DirectionalPose2d getTargetPose() {
        return targetPose;
    }

    public void clear() {
        targetPose = null;
    }

    public Optional<HolonomicDriveSignal> update(
            Pose2d position,
            HolonomicDriveSignal inputDriveSignal) {
        if (targetPose == null) {
            return Optional.empty();
        } else {
            double x = inputDriveSignal.getTranslation().getX();
            double y = inputDriveSignal.getTranslation().getY();
            if (targetPose.isXRestricted()) {
                x = xController.calculate(position.getX(), targetPose.getX());
            }
            if (targetPose.isYRestricted()) {
                y = yController.calculate(position.getY(), targetPose.getY());
            }
            return Optional.of(
                    new HolonomicDriveSignal(
                            new Translation2d(x, y),
                            0.0,
                            true,
                            true));
        }
    }

    public boolean isXRestricted() {
        return (targetPose == null ? false : targetPose.isXRestricted());
    }

    public boolean isYRestricted() {
        return (targetPose == null ? false : targetPose.isYRestricted());
    }
}
