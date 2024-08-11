package frc.robot.utils.shooting;
import org.frcteam6941.looper.Updatable;

import com.team254.lib.util.Util;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.display.Display;
import frc.robot.display.OperatorDashboard;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldLayout;
import frc.robot.utils.TunableNumber;
import org.frcteam6941.looper.Updatable;

public class ShootingDecider implements Updatable {
    public static final Translation2d kCornerTarget = new Translation2d(1.0, FieldLayout.kFieldWidth - 1.5);
    public static final Translation2d kMidTarget = new Translation2d((FieldLayout.kFieldLength / 2.0) - 1.0,
            FieldLayout.kFieldWidth - 0.2);
    // Ferry Related
    private static final double kOppoWingToAllianceWall = FieldLayout.distanceFromAllianceWall(FieldLayout.kWingX,
            true);
    private static ShootingDecider instance;
    TunableNumber ampAngle;
    TunableNumber ampVelocity;
    LaunchParameterTable speakerParams;
    LaunchParameterTable highFerryParams;
    LaunchParameterTable lowFerryParams;

    private ShootingDecider() {
        speakerParams = new LaunchParameterTable("Speaker");
        highFerryParams = new LaunchParameterTable("HighFerry");
        lowFerryParams = new LaunchParameterTable("Low Ferry");

        ampAngle = new TunableNumber("Amp Angle Degs", 165.0);
        ampVelocity = new TunableNumber("Amp Velocity Rpm", -3000.0);

        speakerParams.loadParameter(1.064, -4500, 0.2574 / 3.14 * 180);// 20240808
        speakerParams.loadParameter(1.245, -4500, 0.5026 / 3.14 * 180);// 20240808
        speakerParams.loadParameter(1.600, -4500, 0.6736 / 3.14 * 180);// 20240808
        speakerParams.loadParameter(1.910, -4500, 0.8587 / 3.14 * 180);// 20240808
        speakerParams.loadParameter(2.20, -4500, 0.9205 / 3.14 * 180);// 20240808
        speakerParams.loadParameter(2.51, -4500, 0.9704 / 3.14 * 180);// 20240808
        speakerParams.loadParameter(2.8, -4500, 1.0502 / 3.14 * 180);
        speakerParams.loadParameter(3.06, -5700, 1.1117 / 3.14 * 180);// 20240808
        speakerParams.loadParameter(3.47, -5700, 1.1746 / 3.14 * 180);// 20240808
        speakerParams.loadParameter(3.79, -5700, 1.2322 / 3.14 * 180);//
        speakerParams.loadParameter(3.9, -5700, 1.2391 / 3.14 * 180);// 20240808
        speakerParams.ready();

        highFerryParams.loadParameter(1.0, -4000.0, 15.0);
        highFerryParams.loadParameter(3.0, -5000.0, 15.0);
        highFerryParams.ready();
        lowFerryParams.loadParameter(1.5, -2000.0, 15.0);
        lowFerryParams.loadParameter(3.0, -5000.0, 15.0);
        lowFerryParams.ready();
    }

    public static ShootingDecider getInstance() {
        if (instance == null) {
            instance = new ShootingDecider();
        }
        return instance;
    }

    private static boolean inHighFerryZone(Pose2d robot_pose, boolean is_red_alliance) {
        double x = robot_pose.getTranslation().getX();
        double y = robot_pose.getTranslation().getY();
        com.team254.lib.geometry.Translation2d cor_0 = FieldLayout.handleAllianceFlip(
                new com.team254.lib.geometry.Translation2d(FieldLayout.kWingX, 0.0), is_red_alliance);
        com.team254.lib.geometry.Translation2d cor_1 = FieldLayout.kCenterNote2;
        boolean in_x = Util.inRange(x, Math.min(cor_0.x(), cor_1.x()), Math.max(cor_0.x(), cor_1.x()));
        boolean in_y = Util.inRange(y, Math.min(cor_0.y(), cor_1.y()), Math.max(cor_0.y(), cor_1.y()));
        return in_x && in_y;
    }

    private static boolean useMidfieldTarget(double x_coord, boolean is_red_alliance) {
        return FieldLayout.distanceFromAllianceWall(x_coord, is_red_alliance) - kOppoWingToAllianceWall > 1.0;
    }

    @Override
    public void update(double time, double dt) {
        speakerParams.update();
        highFerryParams.update();
        lowFerryParams.update();
    }

    public ShootingParameters getShootingParameter(Destination destination, Pose2d robotPose) {
        Translation2d target, delta;
        Pair<Double, Double> launchParam;

        switch (destination) {
            case AMP:
                return new ShootingParameters(Double.NaN, ampVelocity.get(), ampAngle.get(), new Rotation2d());
            case FERRY:
                boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;
                boolean useMid = inHighFerryZone(robotPose, isRed)
                        || useMidfieldTarget(robotPose.getX(), isRed);
                target = useMid ? AllianceFlipUtil.apply(kMidTarget) : AllianceFlipUtil.apply(kCornerTarget);
                Display.getInstance().setFerryLocation(target);
                delta = target.minus(robotPose.getTranslation());
                OperatorDashboard.getInstance().updateDistanceToTarget(delta.getNorm());
                launchParam = speakerParams.getParameters(delta.getNorm());
                return new ShootingParameters(delta.getNorm(), launchParam.getFirst(), launchParam.getSecond(),
                        new Rotation2d(delta.getX(), delta.getY()).rotateBy(Rotation2d.fromDegrees(180.0)));
            case SPEAKER:
                target = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).toTranslation2d();
                delta = target.minus(robotPose.getTranslation());
                OperatorDashboard.getInstance().updateDistanceToTarget(delta.getNorm());
                launchParam = speakerParams.getParameters(delta.getNorm());
                return new ShootingParameters(delta.getNorm(), launchParam.getFirst(), launchParam.getSecond(),
                        new Rotation2d(delta.getX(), delta.getY()).rotateBy(Rotation2d.fromDegrees(180.0)));
            default:
                throw new IllegalArgumentException("Illegal destination: undefined.");
        }
    }

    public enum Destination {
        AMP, SPEAKER, FERRY
    }
}
