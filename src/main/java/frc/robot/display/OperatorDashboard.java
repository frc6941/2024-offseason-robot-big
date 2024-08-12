package frc.robot.display;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.utils.shooting.ShootingDecider.Destination;
import lombok.Getter;

@Getter
public class OperatorDashboard {
    private static OperatorDashboard instance;
    private final ShuffleboardTab operatorTab;
    private final GenericEntry distanceToTarget, horizontalDistanceToTarget;
    private final GenericEntry useFerry, useAmp, useSpeaker;
    private final GenericEntry isShooterReady, isDrivetrainReady, isArmReady, isReady;
    private final GenericEntry noteAtIntaker, noteAtIndexer, noteAtShooter;
    // TargetStatus
    @Getter
    private Destination currDestination = Destination.SPEAKER;

    private OperatorDashboard() {
        operatorTab = Shuffleboard.getTab("Operator");
        distanceToTarget = operatorTab
                .add("Distance", -1.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        horizontalDistanceToTarget = operatorTab
                .add("Horizontal Distance", -1.0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();

        useFerry = operatorTab
                .add("Use Ferry", false)
                .withPosition(4, 0)
                .withSize(1, 1)
                .getEntry();
        useAmp = operatorTab
                .add("Use Amp", false)
                .withPosition(6, 0)
                .withSize(1, 1)
                .getEntry();
        useSpeaker = operatorTab
                .add("Use Speaker", false)
                .withPosition(5, 0)
                .withSize(1, 1)
                .getEntry();

        isReady = operatorTab
                .add("Ready", false)
                .withPosition(4, 1)
                .withSize(3, 2)
                .getEntry();
        isShooterReady = operatorTab
                .add("Shooter Ready", false)
                .withPosition(4, 3)
                .withSize(1, 1)
                .getEntry();
        isDrivetrainReady = operatorTab
                .add("Drivetrain Ready", false)
                .withPosition(6, 3)
                .withSize(1, 1)
                .getEntry();
        isArmReady = operatorTab
                .add("Arm Ready", false)
                .withPosition(5, 3)
                .withSize(1, 1)
                .getEntry();

        noteAtIntaker = operatorTab
                .add("At Intaker", false)
                .withPosition(4, 4)
                .withSize(1, 1)
                .getEntry();
        noteAtIndexer = operatorTab
                .add("At Indexer", false)
                .withPosition(6, 4)
                .withSize(1, 1)
                .getEntry();
        noteAtShooter = operatorTab
                .add("At Shooter", false)
                .withPosition(5, 4)
                .withSize(1, 1)
                .getEntry();

        updateDestination(currDestination);
    }

    public static OperatorDashboard getInstance() {
        if (instance == null) {
            instance = new OperatorDashboard();
        }
        return instance;
    }

    public void registerAutoSelector(SendableChooser<String> selector) {
        operatorTab
                .add("Auto Selector", selector)
                .withSize(2, 1)
                .withPosition(0, 0);
    }

    public void updateShooterReady(boolean isShooterReady) {
        this.isShooterReady.setBoolean(isShooterReady);
        getReadyStatus();
    }

    public void updateDrivetrainReady(boolean isDrivetrainReady) {
        this.isDrivetrainReady.setBoolean(isDrivetrainReady);
        getReadyStatus();
    }

    public void updateArmReady(boolean isArmReady) {
        this.isArmReady.setBoolean(isArmReady);
        getReadyStatus();
    }

    public void getReadyStatus() {
        this.isReady.setBoolean(
                isShooterReady.get().getBoolean() &&
                        isArmReady.get().getBoolean() &&
                        isDrivetrainReady.get().getBoolean());
    }

    public void updateNotePathStatus(boolean atIntaker, boolean atIndexer, boolean atShooter) {
        this.noteAtIntaker.setBoolean(atIntaker);
        this.noteAtIndexer.setBoolean(atIndexer);
        this.noteAtShooter.setBoolean(atShooter);
    }

    public void updateDestination(Destination dest) {
        currDestination = dest;
        switch (currDestination) {
            case SPEAKER:
                useFerry.setBoolean(false);
                useSpeaker.setBoolean(true);
                useAmp.setBoolean(false);
                break;
            case AMP:
                useFerry.setBoolean(false);
                useSpeaker.setBoolean(false);
                useAmp.setBoolean(true);
                break;
            case FERRY:
                useFerry.setBoolean(true);
                useSpeaker.setBoolean(false);
                useAmp.setBoolean(false);
                break;
            default:
                throw new IllegalArgumentException("illegal destination.");
        }
    }

    public void updateDistanceToTarget(double distanceToTarget) {
        this.distanceToTarget.setDouble(distanceToTarget);
    }

    public void updateHorizontalDistanceToTarget(double distanceToTarget) {
        this.horizontalDistanceToTarget.setDouble(distanceToTarget);
    }
}