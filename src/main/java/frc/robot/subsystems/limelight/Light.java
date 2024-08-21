package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.AllianceFlipUtil;
import lombok.Getter;
import lombok.Setter;
import org.frcteam6941.utils.AngleNormalization;

public class Light extends SubsystemBase {
    private final DigitalOutput light = new DigitalOutput(0);
    private static Light instance;
    @Getter
    @Setter
    private STATE State = STATE.AUTO;

    public Light() {
    }

    public static Light getInstance() {
        if (instance == null) {
            instance = new Light();
        }
        return instance;
    }

    public void lightOff() {
        light.set(true);
        SmartDashboard.putBoolean("light", false);
    }

    public void lightOn() {
        light.set(false);
        SmartDashboard.putBoolean("light", true);
    }

    @Override
    public void periodic() {
        if (this.getState() == STATE.ON || (
                Math.abs(
                        Swerve.getInstance().getGyro().getYaw().getDegrees() -
                                AngleNormalization.placeInAppropriate0To360Scope(Swerve.getInstance().getGyro().getYaw().getDegrees(),
                                AllianceFlipUtil.apply(
                                        Constants.FieldConstants.Speaker.centerSpeakerOpening
                                        .minus(new Translation3d(0.4, 0, 0)))
                                .toTranslation2d()
                                .minus(Swerve.getInstance().getLocalizer().getCoarseFieldPose(0).getTranslation())
                                .rotateBy(Rotation2d.fromDegrees(180)).getAngle().getDegrees())) < 20.0
                        && AllianceFlipUtil.apply(Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)).getX()
                            < Constants.FieldConstants.wingX
                        && this.getState() != STATE.OFF)) {
            Light.getInstance().lightOn();
        } else {
            Light.getInstance().lightOff();
        }
    }

    public enum STATE {
        ON, OFF, AUTO
    }
}
