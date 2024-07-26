package org.frcteam6941.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DirectionalPose2d extends Pose2d{
    private boolean xRestricted;
    private boolean yRestricted;
    private boolean thetaRestricted;

    public DirectionalPose2d(Pose2d pose, boolean xRestricted, boolean yRestricted, boolean thetaRestricted) {
        super(pose.getTranslation(), pose.getRotation());
        this.xRestricted = xRestricted;
        this.yRestricted = yRestricted;
        this.thetaRestricted = thetaRestricted;
    }

    public DirectionalPose2d() {
        super(new Translation2d(), new Rotation2d());
        this.xRestricted = false;
        this.yRestricted = false;
        this.thetaRestricted = false;
    }

    public boolean isXRestricted() {
        return this.xRestricted;
    }

    public void setXRestricted(boolean xRestricted) {
        this.xRestricted = xRestricted;
    }

    public boolean isYRestricted() {
        return this.yRestricted;
    }

    public void setYRestricted(boolean yRestricted) {
        this.yRestricted = yRestricted;
    }

    public boolean isThetaRestricted() {
        return this.thetaRestricted;
    }

    public void setThetaRestricted(boolean thetaRestricted) {
        this.thetaRestricted = thetaRestricted;
    }
    
}
