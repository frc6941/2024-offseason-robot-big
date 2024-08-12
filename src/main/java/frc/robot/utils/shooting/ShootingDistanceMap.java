package frc.robot.utils.shooting;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

public class ShootingDistanceMap {
    public static double[][] skewOffsetPair = {
            // @x --> horizontal offset from target (meters)
            // @y --> angle to add to robot heading (degrees)
            {0.0, 0.0},
            {3.0, 0.0},
            {3.8, 0.0}

    };
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> skewOffsetMap =
            new InterpolatingTreeMap<>();

    static {
        for (double[] pair : skewOffsetPair) {
            skewOffsetMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
    }
}
