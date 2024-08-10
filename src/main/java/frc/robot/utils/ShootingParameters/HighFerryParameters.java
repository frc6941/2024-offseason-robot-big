package frc.robot.utils.ShootingParameters;

import frc.robot.utils.TunableNumber;

import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.TreeMap;

public class HighFerryParameters {
    private static HighFerryParameters instance;
    private final List<ParametersBinding> parameters = new ArrayList<>();
    private final NavigableMap<Double, ShootingParameters> interpolatingTable = new TreeMap<>();

    private HighFerryParameters() {
        readyTuning();
    }

    public static HighFerryParameters getInstance() {
        if (instance == null) {
            instance = new HighFerryParameters();
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
                    new TunableNumber("P" + counter + " Distance", key),
                    new TunableNumber("P" + counter + " Velocity", interpolatingTable.get(key).getVelocity()),
                    new TunableNumber("P" + counter + " Angle", interpolatingTable.get(key).getAngle())));
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
