package frc.robot.display;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

import org.frcteam6941.looper.Updatable;

import java.util.Optional;

public class Display implements Updatable {
    FieldView fieldView;
    Swerve swerve;

    private static Display instance;

    private Display() {
        swerve = Swerve.getInstance();
        fieldView = new FieldView();
    }

    public static Display getInstance() {
        if (instance == null) {
            instance = new Display();
        }
        return instance;
    }

    @Override
    public void update(double time, double dt) {
        fieldView.update(
            swerve.getLocalizer().getCoarseFieldPose(time), 
            swerve.getLocalizer().getPredictedPose(0.1)
        );;
    }
}
