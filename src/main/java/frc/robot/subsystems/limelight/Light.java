package frc.robot.subsystems.limelight;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import lombok.Setter;

public class Light extends SubsystemBase {
    private final DigitalOutput light = new DigitalOutput(0);
    private static Light instance;
    @Getter
    @Setter
    private STATE STATE;

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
    }

    public void lightOn() {
        light.set(false);
    }

    @Override
    public void periodic() {
        if ((this.STATE == STATE.ON ||
                Math.abs(Limelight.getInstance().getSpeakerRelativePosition().getAngle().getDegrees() -
                        Swerve.getInstance().getGyro().getYaw().getDegrees() -
                        Swerve.getInstance().getGyro().getYaw().getDegrees()) < 40.0)
                && this.STATE != STATE.OFF) {
            Light.getInstance().lightOn();
        } else {
            Light.getInstance().lightOff();
        }
    }

    public enum STATE {
        ON, OFF, AUTO
    }
}
