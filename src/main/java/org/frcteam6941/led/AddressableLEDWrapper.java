package org.frcteam6941.led;

import org.frcteam6941.led.patterns.SolidColorPattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.util.Color;

public class AddressableLEDWrapper {
    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    private AddressableLEDPattern pattern = new SolidColorPattern(Color.kBlack);
    private Runnable runnable = new Runnable() {
		@Override
		public void run() {
			update();
		}
	};
    private Notifier looper = new Notifier(runnable);

    private double intensity = 0.1;
    private double periodic = 0.05;

    public AddressableLEDWrapper(int port, int length) {
        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setData(buffer);
        led.start();

        start(periodic);
    }

    public void setIntensity(double intensity) {
        this.intensity = intensity;
    }

    public void setPeriodic(double periodic) {
        this.periodic = periodic;
        stop();
        start(this.periodic);
    }

    public void setPattern(AddressableLEDPattern pattern) {
		this.pattern = pattern;
	}

    public void update() {
        pattern.setLEDs(buffer);
        for (int i = 0; i < buffer.getLength(); i++) {
            Color originalColor = buffer.getLED(i);
            Color intensityAdjustedColor = new Color(originalColor.red * intensity, originalColor.green * intensity,
                    originalColor.blue * intensity);
            buffer.setLED(i, intensityAdjustedColor);
        }
        led.setData(buffer);
    }

    public void start(double periodic) {
        looper.startPeriodic(periodic);
    }

    public void stop() {
        looper.stop();
    }
}
