package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.AllianceFlipUtil;
import lombok.Getter;
import lombok.Setter;

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
                        Math.abs(Limelight.getInstance().getSpeakerRelativePosition().getAngle().getDegrees()
                                - Swerve.getInstance().getGyro().getYaw().getDegrees())
                                - (AllianceFlipUtil.shouldFlip() ? 180 : 0)) < 20.0
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
