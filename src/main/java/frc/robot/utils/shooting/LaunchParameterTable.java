package frc.robot.utils.shooting;

import frc.robot.utils.TunableNumber;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;

import java.util.NavigableMap;
import java.util.TreeMap;

public class LaunchParameterTable {
    private final List<ParametersBinding> parameters = new ArrayList<>();
    private final NavigableMap<Double, Pair<Double, Double>> interpolatingTable = new TreeMap<>();
    private final String prefix;

    public LaunchParameterTable(String prefix) {
        this.prefix = prefix;
        readyTuning();
    }

    private void readyTuning() {
        int counter = 1;
        for (Double key : interpolatingTable.keySet()) {
            parameters.add(new ParametersBinding(
                    new TunableNumber(prefix + counter + " Distance", key),
                    new TunableNumber(prefix + counter + " Velocity", interpolatingTable.get(key).getFirst()),
                    new TunableNumber(prefix + counter + " Angle", interpolatingTable.get(key).getSecond())));
            counter++;
        }
    }

    public void loadParameter(double distance, double velocity, double angle) {
        interpolatingTable.put(distance, new Pair<Double, Double>(velocity, angle));
    }

    public void update() {
        interpolatingTable.clear();
        for (ParametersBinding bind : parameters) {
            interpolatingTable.put(bind.distance.get(),
                    new Pair<Double, Double>(bind.shootingVelocity.get(), bind.shootingAngle.get()));
        }
    }

    public Pair<Double, Double> getParameters(double distance) {
        distance = MathUtil.clamp(distance, interpolatingTable.firstKey().doubleValue(),
                interpolatingTable.lastKey().doubleValue());

        var floor = interpolatingTable.floorEntry(distance);
        var ceiling = interpolatingTable.ceilingEntry(distance);

        double k = (distance - floor.getKey()) / (ceiling.getKey() - floor.getKey());
        return new Pair<Double, Double>(
                floor.getValue().getFirst()
                        + (ceiling.getValue().getFirst() - floor.getValue().getFirst()) * k,
                floor.getValue().getSecond() + (ceiling.getValue().getSecond() - floor.getValue().getSecond()) * k);
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
