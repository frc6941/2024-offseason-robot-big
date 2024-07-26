package org.frcteam6941.utils;

import edu.wpi.first.wpilibj.util.Color;

public class ColorConversions {
    /**
     * Function to change an RGB color to a WPILib Compatible HSV value (0-180, 0-255, 0-255).
     * Reference: https://zhuanlan.zhihu.com/p/128477902
     *
     * @param color The input WPILib Color
     * @return An double array containing the HSV values
     */
    public static double[] fromRGBtoHSV(Color color) {
        double r = color.red;
        double g = color.green;
        double b = color.blue;
        double[] result = new double[3];

        double cMax = Math.max(Math.max(r, g), b);
        double cMin = Math.min(Math.min(r, g), b);
        double delta = cMax - cMin;

        if (delta == 0) {
            result[0] = 0;
        } else {
            if (cMax == r) {
                result[0] = 60 * (((g - b) / delta) + 0);
            } else if (cMax == g) {
                result[0] = 60 * (((b - r) / delta) + 2);
            } else {
                result[0] = 60 * (((r - g) / delta) + 4);
            }
        }
        result[0] = result[0] / 2.0;

        if (cMax == 0) {
            result[1] = 0;
        } else {
            result[1] = delta / cMax;
        }
        result[1] = result[1] * 255.0;

        result[2] = cMax * 255.0;

        return result;
    }

    public static void main(String[] args) {
        System.out.println(fromRGBtoHSV(new Color(1.0, 0.0, 0.0))[0]);
    }
}
