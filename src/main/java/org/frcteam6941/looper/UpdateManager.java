package org.frcteam6941.looper;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class UpdateManager {
	private final Object taskRunningLock_ = new Object();
	public final List<Updatable> updatables = new ArrayList<>();

	private double lastTimestamp = 0.0;

	private Runnable enableRunnable = new Runnable() {
		@Override
		public void run() {
			synchronized (taskRunningLock_) {
				updatables.forEach(s -> {
					double fpgaTime = Timer.getFPGATimestamp();
					final double timestamp = fpgaTime > 10e-5 ? fpgaTime : lastTimestamp;
					final double dt = timestamp - lastTimestamp > 10e-5 ? timestamp - lastTimestamp
							: Constants.LOOPER_DT;
					lastTimestamp = timestamp;
					s.read(timestamp, dt);
					s.update(timestamp, dt);
					s.write(timestamp, dt);
					s.telemetry();
				});
			}
		}
	};

	private final Runnable simulationRunnable = () -> {
		synchronized (taskRunningLock_) {
			updatables.forEach(s -> {
				double fpgaTime = Timer.getFPGATimestamp();
				final double timestamp = fpgaTime != 0.0 ? fpgaTime : lastTimestamp;
				final double dt = timestamp - lastTimestamp > 10e-5 ? timestamp - lastTimestamp
						: Constants.LOOPER_DT;
				lastTimestamp = timestamp;
				s.simulate(timestamp, dt);
				s.update(timestamp, dt);
				s.write(timestamp, dt);
				s.telemetry();
			});
		}
	};

	private final Notifier updaterEnableThread = new Notifier(enableRunnable);
	private final Notifier updaterSimulationThread = new Notifier(simulationRunnable);

	public UpdateManager(Updatable... updatables) {
		this(Arrays.asList(updatables));
	}

	public UpdateManager(List<Updatable> updatables) {
		this.updatables.addAll(updatables);
	}

	public void startEnableLoop(double period) {
		updaterEnableThread.startPeriodic(period);
	}

	public void runEnableSingle() {
		enableRunnable.run();
	}

	public void stopEnableLoop() {
		updaterEnableThread.stop();
	}

	public void startSimulateLoop(double period) {
		updaterSimulationThread.startPeriodic(period);
	}

	public void runSimulateSingle() {
		simulationRunnable.run();
	}

	public void stopSimulateLoop() {
		updaterSimulationThread.stop();
	}

	public void invokeStart() {
		updatables.forEach(Updatable::start);
	}

	public void invokeStop() {
		updatables.forEach(Updatable::stop);
	}

	public void registerAll() {
		updatables.forEach((Updatable u) -> {
			if (u instanceof Subsystem) {
				Subsystem adapted = (Subsystem) u;
				CommandScheduler.getInstance().registerSubsystem(adapted);
			}
		});
	}
}
