package frc.robot.display;

import org.frcteam6941.looper.Updatable;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.AllianceFlipUtil;

public class Display implements Updatable {
    FieldView fieldView;
    Swerve swerve;
    OperatorDashboard dashboard;

    private static Display instance;

    private Display() {
        swerve = Swerve.getInstance();
        fieldView = new FieldView();
        dashboard = OperatorDashboard.getInstance();
    }

    public static Display getInstance() {
        if (instance == null) {
            instance = new Display();
        }
        return instance;
    }

    public void setFerryLocation(Translation2d pos) {
        fieldView.setFerryLocation(pos);
    }

    @Override
    public void update(double time, double dt) {
        fieldView.update(
            swerve.getLocalizer().getCoarseFieldPose(time), 
            swerve.getLocalizer().getPredictedPose(0.02)
        );
    }
}
