package frc.robot.a;

/**
 * Used to limit the output rate
 */
public class Timer {
	public Timer() {}

	private static int cnt = 0;
	private static int outputRate = 50;

	public void UpdateTimer()
	{
		cnt++;
	}

	public void SetOutputRate(int outputRate)
	{
		this.outputRate = outputRate;
	}

	public boolean Output()
	{
		if (cnt%outputRate == 0)
		{
			cnt -= outputRate;
			return true;
		}
		return false;
	}
}
