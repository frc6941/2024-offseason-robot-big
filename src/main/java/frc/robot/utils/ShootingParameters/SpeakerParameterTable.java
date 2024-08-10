package frc.robot.utils.ShootingParameters;

import frc.robot.utils.TunableNumber;

import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.TreeMap;

public class SpeakerParameterTable {
    private static SpeakerParameterTable instance;
    private final List<ParametersBinding> parameters = new ArrayList<>();
    private final NavigableMap<Double, ShootingParameters> interpolatingTable = new TreeMap<>();

    private SpeakerParameterTable() {
        // loadParameter(1.10, -4500, 0.4494 / 3.14 * 180);//20240807
        // loadParameter(1.68, -4500, 0.6427 / 3.14 * 180);//20240807
        // loadParameter(2.01, -4500, 0.7000 / 3.14 * 180);//20240807
        // loadParameter(2.36, -4500, 0.8513 / 3.14 * 180);//20240807
        // loadParameter(2.60, -4500, 0.9556 / 3.14 * 180);//20240807
        // loadParameter(2.92, -4500, 0.9924 / 3.14 * 180);//20240807
        // loadParameter(3.40, -5700, 1.0630 / 3.14 * 180);//20240807
        // loadParameter(3.67, -5700, 1.2115 / 3.14 * 180);//20240807
        // loadParameter(4.00, -5700, 1.2532 / 3.14 * 180);//20240807
        loadParameter(1.064, -4500, 0.2574 / 3.14 * 180);//20240808
        loadParameter(1.245, -4500, 0.5026 / 3.14 * 180);//20240808
        loadParameter(1.600, -4500, 0.6736 / 3.14 * 180);//20240808
        loadParameter(1.910, -4500, 0.8587 / 3.14 * 180);//20240808
        loadParameter(2.20, -4500, 0.9205 / 3.14 * 180);//20240808
        loadParameter(2.51, -4500, 0.9704 / 3.14 * 180);//20240808
        loadParameter(2.8, -4500, 1.0502 / 3.14 * 180);
        loadParameter(3.06, -5700, 1.1117 / 3.14 * 180);//20240808
        loadParameter(3.47, -5700, 1.1746 / 3.14 * 180);//20240808
        loadParameter(3.79, -5700, 1.2322 / 3.14 * 180);//
        loadParameter(3.9, -5700, 1.2391 / 3.14 * 180);//20240808
        readyTuning();
    }

    public static SpeakerParameterTable getInstance() {
        if (instance == null) {
            instance = new SpeakerParameterTable();
        }
        return instance;
    }

    private void loadParameter(double distance, double velocity, double angle) {
        interpolatingTable.put(distance, new ShootingParameters(velocity, angle));
    }

    private void readyTuning() {
        int counter = 1;
        for (Double key : interpolatingTable.keySet()) {
            parameters.add(new ParametersBinding(
                    new TunableNumber("Speaker" + counter + " Distance", key),
                    new TunableNumber("Speaker" + counter + " Velocity", interpolatingTable.get(key).getVelocity()),
                    new TunableNumber("Speaker" + counter + " Angle", interpolatingTable.get(key).getAngle())));
            counter++;
        }
    }

    public void update() {
        interpolatingTable.clear();
        for (ParametersBinding bind : parameters) {
            interpolatingTable.put(bind.distance.get(),
                    new ShootingParameters(bind.shootingVelocity.get(), bind.shootingAngle.get()));
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
                floor.getValue().getVelocity() + (ceiling.getValue().getVelocity() - floor.getValue().getVelocity()) * k,
                floor.getValue().getAngle() + (ceiling.getValue().getAngle() - floor.getValue().getAngle()) * k);
    }

    public double getFarthestDistance() {
        return interpolatingTable.lastKey();
    }

    private class ParametersBinding implements Comparable<ParametersBinding> {
        public TunableNumber distance;
        public TunableNumber shootingAngle;
        public TunableNumber shootingVelocity;

        public ParametersBinding(TunableNumber distance, TunableNumber velocity, TunableNumber angle) {
            this.distance = distance;
            this.shootingAngle = angle;
            this.shootingVelocity = velocity;
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
