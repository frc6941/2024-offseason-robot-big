package frc.robot.commands;

import com.team254.lib.util.PolynomialRegression;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import org.frcteam1678.lib.math.Conversions;

import java.util.ArrayList;

public class CharacterizationDriveCommand extends Command {
    private final double maxVoltage;
    private final double prepTime = 3.0;
    private final Timer prepTimer = new Timer();
    private final Timer timer = new Timer();
    private final ArrayList<Double> yVoltages = new ArrayList<>();
    private final ArrayList<Double> xVelocities = new ArrayList<>();
    private final ArrayList<Double> xFalconVelocities = new ArrayList<>();
    private Swerve drivetrain;
    private double startVoltage;
    private double deltaVoltage;
    private final Runnable r = () -> {
        if (prepTimer.get() < prepTime) {
            timer.stop();
            SwerveModuleState individualState = new SwerveModuleState(
                    0.0, new Rotation2d());
            drivetrain.setModuleStates(new SwerveModuleState[]{
                    individualState, individualState, individualState, individualState
            }, true, true);
            return;
        }

        timer.start();
        double targetVoltage = startVoltage + deltaVoltage * timer.get();

        SwerveModuleState individualState = new SwerveModuleState(
                targetVoltage / 1.0, new Rotation2d()
        );
        System.out.println("REQ = " + individualState.toString());
        drivetrain.setModuleStates(new SwerveModuleState[]{
                individualState,
                individualState,
                individualState,
                individualState
        }, true, true);
        yVoltages.add(targetVoltage);

        SwerveModuleState[] moduleStates = drivetrain.getModuleStates();
        double averageVelocity = 0.0;
        double averageFalconVelocity = 0.0;
        for (SwerveModuleState state : moduleStates) {
            averageVelocity += Math.abs(state.speedMetersPerSecond);
            averageFalconVelocity += Math.abs(Conversions.MPSToFalcon(
                    state.speedMetersPerSecond,
                    Constants.SwerveDrivetrain.wheelCircumferenceMeters.magnitude(),
                    Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO)
            );
        }
        averageVelocity /= moduleStates.length;
        averageFalconVelocity /= moduleStates.length;
        xVelocities.add(averageVelocity);
        xFalconVelocities.add(averageFalconVelocity);

    };
    private final Notifier n = new Notifier(r);
    private double travelTicks = 0;

    public CharacterizationDriveCommand(Swerve drivetrain, double startVoltage, double deltaVoltage, double maxVoltage) {
        this.drivetrain = drivetrain;
        this.startVoltage = startVoltage;
        this.deltaVoltage = deltaVoltage;
        this.maxVoltage = maxVoltage;
    }

    @Override
    public void initialize() {
        prepTimer.reset();
        timer.reset();
        drivetrain.empty();
        System.out.println("--- Linear Characterization of the Drivetrain Starts ---");
        prepTimer.start();
        n.startPeriodic(0.01);
    }

    @Override
    public void end(boolean interrupted) {
        n.stop();
        drivetrain.stopMovement();
        drivetrain.normal();

        System.out.println("--- Linear Characterization of the Drivetrain Ends ---");
        System.out.println("Total Time Taken: " + timer.get());
        prepTimer.reset();
        timer.stop();

        if (xVelocities.isEmpty() || yVoltages.isEmpty() || xFalconVelocities.isEmpty()) return;

        PolynomialRegression regression = new PolynomialRegression(
                xVelocities.stream().mapToDouble(Math::abs).toArray(),
                yVoltages.stream().mapToDouble(Math::abs).toArray(), 1);
        //System.out.println(regression);
        System.out.println("Fit R2: " + regression.R2());
        System.out.println("Drivetrain KS: " + regression.beta(0) + " V");
        System.out.println("Drivetrain kV: " + regression.beta(1) + " V / ms^{-1}");
        System.out.println(
                "Converted Module kV: "
                        + regression.beta(1) / Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO
                        * Constants.SwerveDrivetrain.wheelCircumferenceMeters.magnitude()
                        + " V / rps");

        PolynomialRegression regressionFalcon = new PolynomialRegression(
                xFalconVelocities.stream().mapToDouble(Math::abs).toArray(),
                yVoltages.stream().mapToDouble(Math::abs).toArray(), 1);
        System.out.println(
                "Converted Module kV in Falcon Units:"
                        + 1024.0 * regressionFalcon.beta(0) + "Falcon Output Units / Falcon Encoder Units / 100ms");
        System.out.println(
                "Travelled Ticks: " + travelTicks
        );
    }

    @Override
    public boolean isFinished() {
        return (startVoltage + deltaVoltage * timer.get()) >= maxVoltage;
    }
}