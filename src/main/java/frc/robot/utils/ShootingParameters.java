package frc.robot.utils;

public class ShootingParameters {
	private double shootingVoltage;
	private double shootingAngle;

	public double getVoltage() {
		return shootingVoltage;
	}

	public double getAngle() {
		return shootingAngle;
	}

	public ShootingParameters(double voltage, double angle) {
		this.shootingVoltage = voltage;
		this.shootingAngle = angle;
	}

	@Override
	public String toString() {
		return String.format("<Voltage: %.2f V, Angle: %.2f rad>", shootingVoltage, shootingAngle);
	}
}