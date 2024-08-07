package frc.robot.commands.test;

import com.team254.lib.util.PolynomialRegression;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;

import java.util.ArrayList;

import static edu.wpi.first.units.Units.Volts;

public class CharacterizationShooterCommand extends Command {
    private final double maxVoltage;
    private final double prepTime = 3.0;
    private final Timer prepTimer = new Timer();
    private final Timer timer = new Timer();
    private final ArrayList<Double> yVoltages = new ArrayList<>();
    private final ArrayList<Double> xVelocities = new ArrayList<>();
    ShooterSubsystem shooterSubsystem = new ShooterSubsystem(new ShooterIOTalonFX());
    private double startVoltage;
    private double deltaVoltage;
    private final Runnable r = () -> {
        if (prepTimer.get() < prepTime) {
            timer.stop();
            shooterSubsystem.getIo().setFlyWheelDirectVoltage(Volts.of(0));
            return;
        }

        timer.start();
        double targetVoltage = startVoltage + deltaVoltage * timer.get();

        shooterSubsystem.getIo().setFlyWheelDirectVoltage(Volts.of(-targetVoltage));
        System.out.println("REQ = " + -targetVoltage);
        yVoltages.add(-targetVoltage);

        double averageVelocity = shooterSubsystem.getIo().getVelocity();
        xVelocities.add(averageVelocity);

    };
    private final Notifier n = new Notifier(r);

    public CharacterizationShooterCommand(ShooterSubsystem shooterSubsystem, double startVoltage, double deltaVoltage, double maxVoltage) {
        this.shooterSubsystem = shooterSubsystem;
        this.startVoltage = startVoltage;
        this.deltaVoltage = deltaVoltage;
        this.maxVoltage = maxVoltage;
    }

    @Override
    public void initialize() {
        prepTimer.reset();
        timer.reset();
        System.out.println("--- Linear Characterization of the Drivetrain Starts ---");
        prepTimer.start();
        n.startPeriodic(0.01);
    }

    @Override
    public void end(boolean interrupted) {
        n.stop();
        shooterSubsystem.getIo().setFlyWheelDirectVoltage(Volts.of(0));

        System.out.println("--- Linear Characterization of the Drivetrain Ends ---");
        System.out.println("Total Time Taken: " + timer.get());
        prepTimer.reset();
        timer.stop();

        if (xVelocities.isEmpty() || yVoltages.isEmpty()) return;

        PolynomialRegression regression = new PolynomialRegression(
                xVelocities.stream().mapToDouble(Math::abs).toArray(),
                yVoltages.stream().mapToDouble(Math::abs).toArray(), 1);
        //System.out.println(regression);
        System.out.println("Fit R2: " + regression.R2());
        System.out.println("Drivetrain KS: " + regression.beta(0) + " V");
        System.out.println("Drivetrain kV: " + regression.beta(1) + " V / ms^{-1}");
        System.out.println(
                "Converted Module kV: "
                        + regression.beta(1) * Constants.SwerveDrivetrain.wheelCircumferenceMeters.magnitude()
                        + " V / rps");
    }

    @Override
    public boolean isFinished() {
        return (startVoltage + deltaVoltage * timer.get()) >= maxVoltage;
    }
}