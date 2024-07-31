package com.team254.frc2020.limelight.undistort;

public class CameraConstants {
    private final double[][] cameraMatrix;
    private final double[] cameraDistortion;

    public CameraConstants(double[] cameraDistortion, double[][] cameraMatrix) {
        this.cameraDistortion = cameraDistortion;
        this.cameraMatrix = cameraMatrix;
    }

    public double[][] getCameraMatrix() {
        return cameraMatrix;
    }

    public double[] getCameraDistortion() {
        return cameraDistortion;
    }
}
