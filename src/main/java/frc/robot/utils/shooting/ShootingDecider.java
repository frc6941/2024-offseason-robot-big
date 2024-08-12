package frc.robot.utils.shooting;

import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.FieldConstants;
import frc.robot.display.Display;
import frc.robot.display.OperatorDashboard;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldLayout;
import frc.robot.utils.TunableNumber;
import org.frcteam6941.looper.Updatable;

import static frc.robot.Constants.ShooterConstants.*;

public class ShootingDecider implements Updatable {
    public static final Translation2d kCornerTarget = new Translation2d(1.0, FieldLayout.kFieldWidth - 1.5);
    public static final Translation2d kMidUpperTarget = new Translation2d((FieldLayout.kFieldLength / 2.0) - 1.0,
            FieldLayout.kFieldWidth - 0.2);

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

        highFerryParams.loadParameter(5.0, -2000.0, 10.0);
        highFerryParams.loadParameter(6.5, -2700.0, 10.0);
        highFerryParams.loadParameter(8.0, -4000.0, 10.0);
        highFerryParams.loadParameter(9.5, -5500.0, 10.0);
        highFerryParams.loadParameter(11.0, -6500.0, 10.0);
        highFerryParams.ready();
        lowFerryParams.loadParameter(1.5, -2000.0, 15.0);
        lowFerryParams.loadParameter(3.0, -5000.0, 15.0);
        lowFerryParams.loadParameter(3.0, -5000.0, 15.0);
        lowFerryParams.loadParameter(3.0, -5000.0, 15.0);
        lowFerryParams.loadParameter(3.0, -5000.0, 15.0);
        lowFerryParams.ready();
    }

    public static ShootingDecider getInstance() {
        if (instance == null) {
            instance = new ShootingDecider();
        }
        return instance;
    }

    private static boolean inHighFerryZone(Pose2d robotPose) {
        Pose2d pose = AllianceFlipUtil.apply(robotPose);
        return pose.getX() > FieldConstants.wingOpponentX;
    }

    private static double getSkewCompensationFromRegression(double y_offset) {
        if (useSmartDashboardForSkew) {
            return skewValue.get();
        }
        return ShootingDistanceMap.skewOffsetMap.getInterpolated(new InterpolatingDouble(y_offset)).value;
    }

    @Override
    public void update(double time, double dt) {
        speakerParams.update();
        highFerryParams.update();
        lowFerryParams.update();
    }

    private double[] getAdjustedShootOnMoveParams(
            double uncompensated_yaw, double uncompensated_range, Twist2d robot_velocity) {
        com.team254.lib.geometry.Translation2d polar_velocity = new com.team254.lib.geometry.Translation2d(robot_velocity.dx, robot_velocity.dy)
                .rotateBy(com.team254.lib.geometry.Rotation2d.fromDegrees(uncompensated_yaw));
        double radial = polar_velocity.x();
        double tangential = polar_velocity.y();

        double shot_speed = uncompensated_range / kToFFactor - radial;
        shot_speed = Math.max(0.0, shot_speed);
        double yaw_adj = Units.radiansToDegrees(Math.atan2(-tangential, shot_speed));
        double range_adj = kToFFactor * Math.sqrt(Math.pow(tangential, 2) + Math.pow(shot_speed, 2));
        return new double[]{yaw_adj + uncompensated_yaw, range_adj};
    }

    public ShootingParameters getShootingParameter(Destination destination, Pose2d robotPose) {
        return getShootingParameter(destination, robotPose, new Twist2d(0, 0, 0));
    }

    public ShootingParameters getShootingParameter(Destination destination, Pose2d robotPose, Twist2d robotVelocity) {
        Translation2d target, delta;
        Pair<Double, Double> launchParam;

        switch (destination) {
            case AMP:
                return new ShootingParameters(Double.NaN, ampVelocity.get(), ampAngle.get(), new Rotation2d());
            case FERRY:
                boolean useMid = inHighFerryZone(robotPose);
                target = useMid ? AllianceFlipUtil.apply(kMidUpperTarget) : AllianceFlipUtil.apply(kCornerTarget);
                Display.getInstance().setFerryLocation(target);
                delta = target.minus(robotPose.getTranslation());
                OperatorDashboard.getInstance().updateDistanceToTarget(delta.getNorm());
                launchParam = useMid ? lowFerryParams.getParameters(delta.getNorm()) : highFerryParams.getParameters(delta.getNorm());
                return new ShootingParameters(delta.getNorm(), launchParam.getFirst(), launchParam.getSecond(),
                        new Rotation2d(delta.getX(), delta.getY()).rotateBy(Rotation2d.fromDegrees(180.0)));
            case SPEAKER:
                target = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).toTranslation2d();
                delta = target.minus(robotPose.getTranslation());

                com.team254.lib.geometry.Translation2d deltaMod = new com.team254.lib.geometry.Translation2d(
                        delta
                );

                double dist = deltaMod.norm();
                double yaw = deltaMod.direction().getDegrees();

                if (useShootOnMove) {
                    double[] somParams = getAdjustedShootOnMoveParams(yaw, dist, robotVelocity);
                    yaw = somParams[0];
                    dist = somParams[1];
                }

                OperatorDashboard.getInstance().updateDistanceToTarget(dist);
                OperatorDashboard.getInstance().updateHorizontalDistanceToTarget(deltaMod.y());
                launchParam = speakerParams.getParameters(dist);

                return new ShootingParameters(dist, launchParam.getFirst(), launchParam.getSecond(),
                        new Rotation2d(yaw)
                                .rotateBy(Rotation2d.fromDegrees(180.0))
                                .rotateBy(Rotation2d.fromDegrees(getSkewCompensationFromRegression(deltaMod.y()))));
            default:
                throw new IllegalArgumentException("Illegal destination: undefined.");
        }
    }

    public enum Destination {
        AMP, SPEAKER, FERRY
    }
}
