package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.TreeMap;

public class ShootingParametersTable {
    private static ShootingParametersTable instance;
    private final List<ParametersBinding> parameters = new ArrayList<>();
    private final NavigableMap<Double, ShootingParameters> interpolatingTable = new TreeMap<>();

    private ShootingParametersTable() {
        loadParameter(1.07, -6, 17.8);
        loadParameter(1.66, -6, 33.8);
		loadParameter(2.33, -8, 38.8);
		loadParameter(2.44, -8, 51.6);
		//loadParameter(2.72, -8, 51.8);
		loadParameter(2.90, -8, 56.8);
        loadParameter(3.17, -8, 57.7);
        loadParameter(3.41, -9, 60.9);
        loadParameter(3.74, -9, 63.9);
        readyTuning();
    }

    public static ShootingParametersTable getInstance() {
        if (instance == null) {
            instance = new ShootingParametersTable();
        }
        return instance;
    }

    private void loadParameter(double distance, double voltage, double angle) {
        interpolatingTable.put(distance, new ShootingParameters(voltage, angle));
    }

    private void readyTuning() {
        int counter = 1;
        for (Double key : interpolatingTable.keySet()) {
            parameters.add(new ParametersBinding(
                    new TunableNumber("P" + counter + " Distance", key),
                    new TunableNumber("P" + counter + " Voltage", interpolatingTable.get(key).getVoltage()),
                    new TunableNumber("P" + counter + " Angle", interpolatingTable.get(key).getAngle())));
            counter++;
        }
    }

    public void update() {
        interpolatingTable.clear();
        for (ParametersBinding bind : parameters) {
            interpolatingTable.put(bind.distance.get(),
                    new ShootingParameters(bind.shootingVoltage.get(), bind.shootingAngle.get()));
        }
    }

	public ShootingParameters getParameters(double distance) {
		if (distance <= interpolatingTable.firstKey()) {
			return interpolatingTable.firstEntry().getValue();
		}

		if (distance >= interpolatingTable.lastKey()) {
			return interpolatingTable.lastEntry().getValue();
		}

		Entry<Double, ShootingParameters> floor = interpolatingTable.floorEntry(distance);
		Entry<Double, ShootingParameters> ceiling = interpolatingTable.ceilingEntry(distance);

		double k = (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey());
		return new ShootingParameters(
				floor.getValue().getVoltage() + (ceiling.getValue().getVoltage() - floor.getValue().getVoltage()) * k,
				floor.getValue().getAngle() + (ceiling.getValue().getAngle() - floor.getValue().getAngle()) * k);
	}
	
	public double getFarthestDistance() {
		return interpolatingTable.lastKey();
	}

    private class ParametersBinding implements Comparable<ParametersBinding> {
        public TunableNumber distance;
        public TunableNumber shootingAngle;
        public TunableNumber shootingVoltage;

        public ParametersBinding(TunableNumber distance, TunableNumber voltage, TunableNumber angle) {
            this.distance = distance;
            this.shootingAngle = angle;
            this.shootingVoltage = voltage;
        }

        @Override
        public int compareTo(ParametersBinding bind) {
            if (distance.get() - bind.distance.get() > 0) {
                return 1;
            } else if (distance.get() - bind.distance.get() < 0) {
                return -1;
            } else {
                return 0;
            }
        }
    }
}
