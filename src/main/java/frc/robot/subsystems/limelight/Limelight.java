package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.AllianceFlipUtil;
import org.frcteam6941.looper.Updatable;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

public class Limelight implements Updatable {
    private static final NetworkTable limelightTable = NetworkTableInstance
            .getDefault()
            .getTable("limelight");

    private static final NetworkTableEntry tv = limelightTable.getEntry("tv");
    private static final NetworkTableEntry tx = limelightTable.getEntry("tx");
    private static final NetworkTableEntry ty = limelightTable.getEntry("ty");
    private static final NetworkTableEntry tid = limelightTable.getEntry("tid");

    private static final NetworkTableEntry botPoseWPIBlue = limelightTable.getEntry("botpose_wpiblue");
    private static final NetworkTableEntry targetPoseCameraSpace = limelightTable.getEntry("targetpose_cameraspace");

    private static final Swerve swerve = Swerve.getInstance();
    private static int measuerCnt = 0;
    private static double deviationX, deviationY, deviationOmega;
    // singleton
    private static Limelight instance;
    private LoggedDashboardNumber distanceLogged = new LoggedDashboardNumber("Distance");
    private Optional<PoseEstimate> botEstimate;

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
                            .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.AIM_LIMELIGHT_NAME));
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
        LimelightHelpers.SetRobotOrientation("limelight",
                Swerve.getInstance().getLocalizer().getLatestPose().getRotation().getDegrees(),
                Swerve.getInstance().getLocalizer().getSmoothedVelocity().getRotation().getDegrees(),
                0, 0, 0, 0);
        //rejection
        if (Swerve.getInstance().getLocalizer().getSmoothedVelocity().getTranslation().getNorm() > Constants.SwerveDrivetrain.maxSpeed.magnitude())
            return;
        if (Math.abs(Swerve.getInstance().getLocalizer().getSmoothedVelocity().getRotation()
                .getDegrees()) > Math.toDegrees(Constants.SwerveDrivetrain.maxAngularRate.magnitude()))
            return;
        // if (Swerve.getInstance().getLocalizer().getLatestPose().getX() < 0
        // 		|| Swerve.getInstance().getLocalizer().getLatestPose().getX() > FieldConstants.fieldLength
        // 		|| Swerve.getInstance().getLocalizer().getLatestPose().getY() < 0
        // 		|| Swerve.getInstance().getLocalizer().getLatestPose().getY() > FieldConstants.fieldWidth)
        // 	return;
        if (!botEstimate.isEmpty()) {
            if (measuerCnt <= 3) {
                measuerCnt++;
                deviationX = 0.01;
                deviationY = 0.01;
                deviationOmega = 0.001;
            } else {
                deviationX = (0.0062 * botEstimate.get().pose.getX() + 0.0087) * 2;
                deviationY = (0.0062 * botEstimate.get().pose.getY() + 0.0087) * 2;
                deviationOmega = 0.001;
            }
            botEstimate.ifPresent((poseEstimate) -> {
                Swerve.getInstance().getLocalizer().addMeasurement(
                        botEstimate.get().timestampSeconds,
                        botEstimate.get().pose,
                        new Pose2d(new Translation2d(deviationX, deviationY), Rotation2d.fromDegrees(deviationOmega)));
            });
        }
    }

    public Translation2d getSpeakerRelativePosition() {
        Pose2d robotPos = swerve.getLocalizer().getCoarseFieldPose(edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
        SmartDashboard.putString("robotPos", robotPos.toString());
        // Pose2d SpeakerPos = new Pose2d(new Translation2d(FieldConstants.Speaker.centerSpeakerOpening.getX(),
        // 		FieldConstants.Speaker.centerSpeakerOpening.getY()), new Rotation2d(0));
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
            distanceLogged.set(Limelight.getInstance().getSpeakerRelativePosition().getNorm());
            SmartDashboard.putString("metaTag2blue",
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.AIM_LIMELIGHT_NAME).pose.toString());
            if (!botEstimate.isEmpty()) {
                SmartDashboard.putString("limelight_pose", botEstimate.get().pose.toString());
                SmartDashboard.putNumber("latency", botEstimate.get().latency);
            }
        }
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