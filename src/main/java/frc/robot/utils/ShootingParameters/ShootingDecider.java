package frc.robot.utils.ShootingParameters;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.utils.FerryUtil;
import frc.robot.utils.TunableNumber;

public class ShootingDecider {
    enum Destination {
        AMP, SPEAKER, FERRY
    }
    
    TunableNumber ampAngle;
    TunableNumber ampVelocity;

    SpeakerParameterTable speakerParams;
    HighFerryParameterTable highFerryParams;
    LowFerryParameterTable lowFerryParams;

    private static ShootingDecider instance;
    public ShootingDecider getInstance() {
        if(instance == null) {
            instance = new ShootingDecider();
        }
        return instance;
    }
    private ShootingDecider() {
        speakerParams = SpeakerParameterTable.getInstance();
        highFerryParams = HighFerryParameterTable.getInstance();
        lowFerryParams = LowFerryParameterTable.getInstance();
        ampAngle = new TunableNumber("Amp Angle Degs", 165.0);
        ampVelocity = new TunableNumber("Amp Velocity Rpm", 3000.0);
    }

    public void update() {
        speakerParams.update();
        highFerryParams.update();
        lowFerryParams.update();
    }

    public ShootingParameters getShootingParameter(Destination destination, Pose2d robotPose) {
        double distance = robotPose.getTranslation().getNorm();
        switch (destination) {
            case AMP:
                return new ShootingParameters(ampVelocity.get(), ampAngle.get());
            case FERRY:
                double[] results = FerryUtil.getFerryShotParameters(robotPose, DriverStation.getAlliance().get() == Alliance.Red);
                return new ShootingParameters(results[1], results[2]);
            case SPEAKER:
                return speakerParams.getParameters(distance);
            default:
                throw new IllegalArgumentException("Illegal destination: undefined.");
        }
    }
}
