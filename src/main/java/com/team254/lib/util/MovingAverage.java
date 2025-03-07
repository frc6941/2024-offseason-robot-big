package com.team254.lib.util;

import java.util.ArrayList;

/**
 * Helper class for storing and calculating a moving average
 */
public class MovingAverage {

    ArrayList<Double> numbers = new ArrayList<>();
    int maxSize;

    public MovingAverage(int maxSize) {
        this.maxSize = maxSize;
    }

    public void addNumber(double newNumber) {
        numbers.add(newNumber);
        if (numbers.size() > maxSize) {
            numbers.remove(0);
        }
    }

    public double getAverage() {
        double total = 0;
        ArrayList<Double> numbersCopy = (ArrayList<Double>) numbers.clone();
        for (double number : numbersCopy) {
            total += number;
        }

        return total / numbersCopy.size();
    }

    public int getSize() {
        return numbers.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    public void clear() {
        numbers.clear();
    }

}
