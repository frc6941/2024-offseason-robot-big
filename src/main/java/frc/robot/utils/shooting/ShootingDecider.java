package frc.robot.utils.shooting;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.FieldConstants;
import frc.robot.display.Display;
import frc.robot.display.OperatorDashboard;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldLayout;
import frc.robot.utils.TunableNumber;
import org.frcteam6941.looper.Updatable;

public class ShootingDecider implements Updatable {
    public static final Translation2d kCornerTarget = new Translation2d(1.0, FieldLayout.kFieldWidth - 1.5);
    public static final Translation2d kMidUpperTarget = new Translation2d(1.0, FieldLayout.kFieldWidth - 1.5);

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
        ampVelocity = new TunableNumber("Amp Velocity Rpm", -1500.0);

        //The first speaker parameter will influence manual shoot, DO NOT CHANGE IT
        speakerParams.loadParameter(1.064, -4500, 14.755);// 20240808
        speakerParams.loadParameter(1.245, -4500, 21.811);// 20240808
        speakerParams.loadParameter(1.600, -4500, 30.614);// 20240808
        speakerParams.loadParameter(1.910, -4500, 40.02);// 20240817
        speakerParams.loadParameter(2.20, -4500, 48.767);// 20240817
        speakerParams.loadParameter(2.51, -4500, 50.000);// 20240817
        speakerParams.loadParameter(2.8, -4500, 56.902);// 20240817
        speakerParams.loadParameter(3.06, -5700, 61.728);// 20240817
        speakerParams.loadParameter(3.47, -5700, 66.333);// 20240817
        speakerParams.loadParameter(3.79, -5700, 69.635);// 20240808
        speakerParams.loadParameter(3.9, -5700, 71.0312);// 20240816
        speakerParams.loadParameter(4.2, -5900, 75.0312);// 20240816
        speakerParams.loadParameter(4.5, -5900, 77.0312);// 20240816
        speakerParams.ready();

        highFerryParams.loadParameter(5.0, -2000.0, 10.0);
        highFerryParams.loadParameter(6.5, -2700.0, 10.0);
        highFerryParams.loadParameter(8.0, -3500.0, 10.0);//20240818
        highFerryParams.loadParameter(9.5, -4650.0, 10.0);//20240818
        highFerryParams.loadParameter(11.0, -5000.0, 10.0);//20240818
        highFerryParams.ready();
        lowFerryParams.loadParameter(1.5, -4850.0, 10.0);
        lowFerryParams.loadParameter(3.1, -4850.0, 10.0);
        lowFerryParams.ready();
    }

    public static ShootingDecider getInstance() {
        if (instance == null) {
            instance = new ShootingDecider();
        }
        return instance;
    }

    public static boolean inHighFerryZone(Pose2d robotPose) {
        Pose2d pose = AllianceFlipUtil.apply(robotPose);
        return pose.getX() > FieldConstants.wingOpponentX;
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
                boolean useMid = inHighFerryZone(robotPose);
                target = useMid ? AllianceFlipUtil.apply(kMidUpperTarget) : AllianceFlipUtil.apply(kCornerTarget);
                Display.getInstance().setFerryLocation(target);
                delta = target.minus(robotPose.getTranslation());
                OperatorDashboard.getInstance().updateDistanceToTarget(delta.getNorm());
                launchParam = useMid ? lowFerryParams.getParameters(delta.getNorm()) : highFerryParams.getParameters(delta.getNorm());
                return new ShootingParameters(delta.getNorm(), launchParam.getFirst(), launchParam.getSecond(),
                        new Rotation2d(delta.getX(), delta.getY()).rotateBy(Rotation2d.fromDegrees(180)));
            case SPEAKER:
                target = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.minus(new Translation3d(0.4, 0, 0))).toTranslation2d();
                delta = target.minus(robotPose.getTranslation());
                OperatorDashboard.getInstance().updateDistanceToTarget(delta.getNorm());
                launchParam = speakerParams.getParameters(delta.getNorm());
                return new ShootingParameters(delta.getNorm(), launchParam.getFirst(), launchParam.getSecond(),
                        new Rotation2d(delta.getX(), delta.getY()).rotateBy(Rotation2d.fromDegrees(180)));
            default:
                throw new IllegalArgumentException("Illegal destination: undefined.");
        }
    }

    public enum Destination {
        AMP, SPEAKER, FERRY
    }
}
