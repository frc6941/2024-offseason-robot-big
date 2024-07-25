package org.frcteam6941.utils;

import com.team254.lib.util.Util;

public class Range {
    public double min;
    public double max;

    public Range(double min, double max) {
        this.min = min;
        this.max = max;
    }

    public boolean inRange(double value) {
        return value >= min && value <= max;
    }

    public double clamp(double value) {
        return Util.clamp(value, min, max);
    }

    public double average() {
        return (min + max) / 2.0;
    }
}
