package org.frcteam6941.localization;

import edu.wpi.first.math.geometry.Pose2d;

public interface Localizer {
    Pose2d getLatestPose();
    Pose2d getPoseAtTime(double time);
    Pose2d getPredictedPose(double lookahead);
    Pose2d getCoarseFieldPose(double time);

    Pose2d getMeasuredVelocity();
    Pose2d getMeasuredAcceleration();
    Pose2d getPredictedVelocity();
    Pose2d getSmoothedVelocity();
    Pose2d getSmoothedPredictedVelocity();
    Pose2d getSmoothedAccleration();

    double getDistanceDriven();

    void addMeasurement(double time, Pose2d measuredDelta, Pose2d stdDeviation);
}
