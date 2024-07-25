package frc.robot.a;

public class Utils {
	public Utils(){}
	public static int sign(double value) {
        if (value < 0) return -1;
        if (value == 0) return 0;
        return 1;
    }
}