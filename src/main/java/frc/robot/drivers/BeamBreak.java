package frc.robot.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

public class BeamBreak {
    private final AnalogInput analogInput;
    private boolean lastStatus;
    private boolean tripped;
    private boolean cleared;

    public BeamBreak(int channel) {
        analogInput = new AnalogInput(channel);
    }

    public void update() {
        boolean value = get();
        tripped = value && !lastStatus;
        cleared = !value && lastStatus;
        lastStatus = value;
    }

    public boolean get() {
        return analogInput.getVoltage() > 1.5;
    }

    public boolean wasTripped() {
        return tripped;
    }

    public boolean wasCleared() {
        return cleared;
    }
}