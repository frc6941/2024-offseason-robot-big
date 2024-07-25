package org.frcteam6941.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometryAdapter {
    public static Pose2d toWpi(com.team254.lib.geometry.Pose2d pose) {
        return new Pose2d(
            new Translation2d(pose.getTranslation().x(), pose.getTranslation().y()),
            new Rotation2d(pose.getRotation().getRadians())
        );
    }

    public static com.team254.lib.geometry.Pose2d to254(Pose2d pose) {
        return new com.team254.lib.geometry.Pose2d(
            pose.getX(), pose.getY(),
            com.team254.lib.geometry.Rotation2d.fromRadians(pose.getRotation().getRadians())
        );
    }
}
