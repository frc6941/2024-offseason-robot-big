package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldLayout;
import org.frcteam6941.looper.Updatable;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class Limelight implements Updatable {
    private static final NetworkTable limelightTable = NetworkTableInstance
            .getDefault()
            .getTable(Constants.VisionConstants.AIM_LIMELIGHT_NAME);

    private static final NetworkTableEntry tv = limelightTable.getEntry("tv");
    private static final NetworkTableEntry tx = limelightTable.getEntry("tx");
    private static final NetworkTableEntry ty = limelightTable.getEntry("ty");
    private static final NetworkTableEntry tid = limelightTable.getEntry("tid");

    private static final NetworkTableEntry botPoseWPIBlue = limelightTable.getEntry("botpose_wpiblue");
    private static final NetworkTableEntry targetPoseCameraSpace = limelightTable.getEntry("targetpose_cameraspace");

    private static final Swerve swerve = Swerve.getInstance();
    private static int measuerCnt = 0;
    private static double deviationX, deviationY, deviationOmega;
    private Translation2d kdeltaToTag;
    private Pose3d kTagPose;

    // singleton
    private static Limelight instance;
    private LoggedDashboardNumber distanceLogged = new LoggedDashboardNumber("Distance");
    private Optional<PoseEstimate> botEstimate;

    private int loopCnt = 0;
    private boolean[][] tagFlagCnt = new boolean[30][60];
    private double[] tagCnt = new double[30];//17

    private Limelight() {
    }

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    /**
     * @return whether there is a target on the camera.
     */
    public static boolean hasTarget() {
        return tv.getDouble(0) == 1;
    }

    /**
     * Get target if present
     *
     * @return Target object
     */
    public static Optional<AprilTagTarget> getTarget() {
        if (!hasTarget())
            return Optional.empty();
        var rawRobotPose = botPoseWPIBlue.getDoubleArray(new double[7]);
        var rawTargetPose = targetPoseCameraSpace.getDoubleArray(new double[6]);
        var tagId = tid.getDouble(-1);
        return Optional.of(
                new AprilTagTarget(
                        tagId,
                        new Translation2d(tx.getDouble(0), ty.getDouble(0)),
                        Microseconds.of(rawRobotPose[6]),
                        new Pose3d(
                                new Translation3d(rawRobotPose[0], rawRobotPose[1], rawRobotPose[2]),
                                new Rotation3d(
                                        Radians.convertFrom(rawRobotPose[3], Degrees),
                                        Radians.convertFrom(rawRobotPose[4], Degrees),
                                        Radians.convertFrom(rawRobotPose[5], Degrees))),
                        new Pose3d(
                                new Translation3d(rawTargetPose[0], rawTargetPose[1], rawTargetPose[2]),
                                new Rotation3d(
                                        Radians.convertFrom(rawTargetPose[3], Degrees),
                                        Radians.convertFrom(rawTargetPose[4], Degrees),
                                        Radians.convertFrom(rawTargetPose[5], Degrees)))));
    }

    @Override
    public void read(double time, double dt) {
        if (hasTarget()) {
            botEstimate = Optional.of(
                    LimelightHelpers
                            .getBotPoseEstimate_wpiBlue(Constants.VisionConstants.AIM_LIMELIGHT_NAME));
            // if (lastPose != null) {
            // 	SmartDashboard.putNumber("degree speed", Math
            // 		.abs((botEstimate.get().pose.getRotation().getDegrees() - lastPose.pose.getRotation().getDegrees())
            // 					/ (botEstimate.get().timestampSeconds - lastPose.timestampSeconds)));
            // 	if(Math
            // 	.abs((botEstimate.get().pose.getRotation().getDegrees() - lastPose.pose.getRotation().getDegrees())
            // 					/ (botEstimate.get().timestampSeconds - lastPose.timestampSeconds)) > 30)
            // 		botEstimate = Optional.empty();
            // }
            //if(!botEstimate.isEmpty()) lastPose = botEstimate.get();
        } else {
            botEstimate = Optional.empty();
        }
    }

    @Override
    public void update(double time, double dt) {
        int speakerTag1 = AllianceFlipUtil.shouldFlip() ? 4 : 7;
        int speakerTag2 = AllianceFlipUtil.shouldFlip() ? 3 : 8;
        loopCnt++;
        int ktagID = (int) LimelightHelpers.getFiducialID(Constants.VisionConstants.AIM_LIMELIGHT_NAME);
        for (int i = 1; i <= 16; i++) {
            if (tagFlagCnt[i][loopCnt]) tagCnt[i]--;
            tagFlagCnt[i][loopCnt] = false;
        }
//        if (botEstimate.isPresent()) {
//            for (LimelightHelpers.RawFiducial rawFiducial : botEstimate.get().rawFiducials) {
//                if (rawFiducial != null) {
//                    tagFlagCnt[rawFiducial.id][loopCnt] = true;
//                    tagCnt[rawFiducial.id]++;
//                } else {
//                    System.out.println("wtf");
//                }
//            }
//        }
//        LimelightHelpers.setPriorityTagID(Constants.VisionConstants.AIM_LIMELIGHT_NAME, 7);
//        ktagID = (int) LimelightHelpers.getFiducialID(Constants.VisionConstants.AIM_LIMELIGHT_NAME);
//        if (FieldLayout.kTagMap.getTagPose(ktagID).isPresent()) {
//            tagFlagCnt[ktagID][loopCnt] = true;
//            tagCnt[ktagID]++;
//        }
//        LimelightHelpers.setPriorityTagID(Constants.VisionConstants.AIM_LIMELIGHT_NAME, 8);
//        ktagID = (int) LimelightHelpers.getFiducialID(Constants.VisionConstants.AIM_LIMELIGHT_NAME);
//        if (FieldLayout.kTagMap.getTagPose(ktagID).isPresent()) {
//            tagFlagCnt[ktagID][loopCnt] = true;
//            tagCnt[ktagID]++;
//        }
        if (ktagID == speakerTag1 || ktagID == speakerTag2) {
            tagFlagCnt[ktagID][loopCnt] = true;
            tagCnt[ktagID]++;
            if (botEstimate.isPresent()) {
                if (botEstimate.get().rawFiducials.length >= 2) {
                    ktagID = ktagID == speakerTag1 ? speakerTag2 : speakerTag1;
                    tagFlagCnt[ktagID][loopCnt] = true;
                    tagCnt[ktagID]++;
                    ktagID = ktagID == speakerTag1 ? speakerTag2 : speakerTag1;
                }
            }

        }
        loopCnt %= 20;
        SmartDashboard.putNumberArray("Limelight/tagCnt", tagCnt);
        SmartDashboard.putNumber("Limelight/tagCnt3", tagCnt[3]);
        SmartDashboard.putNumber("Limelight/loopCnt", loopCnt);
        boolean isAutoDrive = Swerve.getInstance().getState() == Swerve.State.PATH_FOLLOWING;
        double rejectionRange;
        if (isAutoDrive) {
            rejectionRange = 100;
        } else rejectionRange = 3.1;

        LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.AIM_LIMELIGHT_NAME,
                Swerve.getInstance().getLocalizer().getLatestPose().getRotation().getDegrees(),
                Swerve.getInstance().getLocalizer().getSmoothedVelocity().getRotation().getDegrees(),
                0, 0, 0, 0);
        //rejection
        if (Swerve.getInstance().getLocalizer().getSmoothedVelocity().getTranslation().getNorm() > Constants.SwerveConstants.maxSpeed.magnitude())
            return;
        if (Math.abs(Swerve.getInstance().getLocalizer().getSmoothedVelocity().getRotation()
                .getDegrees()) > Math.toDegrees(Constants.SwerveConstants.maxAngularRate.magnitude()))
            return;

        if (FieldLayout.kTagMap.getTagPose(ktagID).isPresent() && botEstimate.isPresent()) {
            kTagPose = FieldLayout.kTagMap.getTagPose(ktagID).get();
            kdeltaToTag = new Translation2d(kTagPose.getX(), kTagPose.getY()).minus(botEstimate.get().pose.getTranslation());
            if (/*kdeltaToTag.getNorm() > rejectionRange ||*/
                    (((tagCnt[speakerTag1] < 15 || tagCnt[speakerTag2] < 15)
                            && !(tagCnt[speakerTag1] == 0 && tagCnt[speakerTag2] == 20 && kdeltaToTag.getNorm() < 2.5)
                            && !(tagCnt[speakerTag2] == 0 && tagCnt[speakerTag1] == 20 && kdeltaToTag.getNorm() < 2.5)
//                            && kdeltaToTag.getNorm() > 2.0
                            && (ktagID == speakerTag1 || ktagID == speakerTag2)))) {
                SmartDashboard.putBoolean("TargetUpdated", false);
                return;
            }
        }
        if (isAutoDrive) {
            return;
        }


        // if (Swerve.getInstance().getLocalizer().getLatestPose().getX() < 0
        // 		|| Swerve.getInstance().getLocalizer().getLatestPose().getX() > Constants.FieldConstants.fieldLength
        // 		|| Swerve.getInstance().getLocalizer().getLatestPose().getY() < 0
        // 		|| Swerve.getInstance().getLocalizer().getLatestPose().getY() > Constants.FieldConstants.fieldWidth)
        // 	return;
        if (botEstimate.isPresent()) {
            if (measuerCnt <= 3) {
                measuerCnt++;
                deviationX = 0.01;
                deviationY = 0.01;
                deviationOmega = 0.001;
            } else if (swerve.getState() == Swerve.State.PATH_FOLLOWING) {
                deviationX = (0.0062 * botEstimate.get().pose.getX() + 0.0087) * 200;//140
                deviationY = (0.0062 * botEstimate.get().pose.getY() + 0.0087) * 200;
                deviationOmega = 15;//(0.0062 * botEstimate.get().pose.getRotation().getDegrees() + 0.0087) * 0.2;
            } else {
                deviationX = (0.0062 * botEstimate.get().pose.getX() + 0.0087) * 40;//80
                deviationY = (0.0062 * botEstimate.get().pose.getY() + 0.0087) * 40;
                deviationOmega = 15;//(0.0062 * botEstimate.get().pose.getRotation().getDegrees() + 0.0087) * 0.2;
            }
            botEstimate.ifPresent((poseEstimate) -> {
                Swerve.getInstance().getLocalizer().addMeasurement(
                        botEstimate.get().timestampSeconds,
                        botEstimate.get().pose,
                        new Pose2d(new Translation2d(deviationX, deviationY),
                                Rotation2d.fromDegrees(deviationOmega)));
                SmartDashboard.putBoolean("TargetUpdated", true);

            });
        }
    }

    public Translation2d getSpeakerRelativePosition() {
        Pose2d robotPos = swerve.getLocalizer().getCoarseFieldPose(edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
        SmartDashboard.putString("robotPos", robotPos.toString());
        // Pose2d SpeakerPos = new Pose2d(new Translation2d(Constants.FieldConstants.Speaker.centerSpeakerOpening.getX(),
        // 		Constants.FieldConstants.Speaker.centerSpeakerOpening.getY()), new Rotation2d(0));
        Translation2d blueSpeakerTranslation = FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d();
        Pose2d blueSpeaker = new Pose2d(blueSpeakerTranslation, Rotation2d.fromDegrees(180.0));
        Translation2d redSpeakerTranslation = new Translation2d(
                FieldConstants.fieldLength - blueSpeakerTranslation.getX(), blueSpeakerTranslation.getY());
        Pose2d redSpeaker = new Pose2d(redSpeakerTranslation, Rotation2d.fromDegrees(0.0));
        Pose2d SpeakerPos = AllianceFlipUtil.shouldFlip() ? redSpeaker : blueSpeaker;
        SmartDashboard.putString("SpeakerPos", SpeakerPos.toString());
        Translation2d relativePos = new Translation2d(
                robotPos.getTranslation().getX() - SpeakerPos.getTranslation().getX(),
                robotPos.getTranslation().getY() - SpeakerPos.getTranslation().getY());
        SmartDashboard.putNumber("headingtarget", relativePos.getAngle().getDegrees() - swerve.getGyro().getYaw().getDegrees());
        return relativePos;
    }

    @Override
    public void write(double time, double dt) {
    }

    @Override
    public void telemetry() {
        //SmartDashboard.putBoolean("has_target", hasTarget());
        if (hasTarget()) {
//            SmartDashboard.putString("metaTag2blue",
//                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.AIM_LIMELIGHT_NAME).pose.toString());
            if (botEstimate.isPresent()) {
                SmartDashboard.putString("limelight_pose", botEstimate.get().pose.toString());
                SmartDashboard.putNumber("latency", botEstimate.get().latency);
                SmartDashboard.putNumber("Limelight/botEstimate", botEstimate.get().rawFiducials.length);
//                LimelightHelpers.printPoseEstimate(botEstimate.get());
            }
        }
        distanceLogged.set(Limelight.getInstance().getSpeakerRelativePosition().getNorm());
        //System.out.println(tx.getDouble(0));
        // SmartDashboard.putNumber("relativePos2",  getSpeakerRelativePosition());
    }

    @Override
    public void start() {
    }

    @Override
    public void stop() {
    }

    public record AprilTagTarget(
            double tagId,
            Translation2d position,
            Measure<Time> latency,
            Pose3d botPoseWPIBlue,
            Pose3d targetPoseCameraSpace) {

    }

}