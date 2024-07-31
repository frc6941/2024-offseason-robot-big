package org.frcteam6941.utils;

public class TimeDelayedBooleanSimulatable {
    private boolean value = false;
    private Double startRecord = null;
    private double currentTime = 0.0;
    
    public boolean update(boolean status, double timeout) {
        if(!value && status) {
            startRecord = currentTime;
        }
        value = status;
        return status && (currentTime - startRecord) > timeout;
    }

    public void updateTime(double currentTime) {
        this.currentTime = currentTime;
    }

    public void reset() {
        value = false;
        startRecord = null;
        currentTime = 0.0;
    }

    @Override
    public String toString() {
        return String.format("Value: %s; Start Time: %.2f; Current Time: %.2f", value, startRecord, currentTime);
    }
}
