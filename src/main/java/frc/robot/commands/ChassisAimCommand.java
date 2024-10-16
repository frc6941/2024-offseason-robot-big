package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;
import frc.robot.utils.shooting.ShootingParameters;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ChassisAimCommand extends Command {
    private final Swerve swerve;
    private final DoubleSupplier driverX;
    private final DoubleSupplier driverY;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;
    private final boolean isAuto;
    private final LoggedDashboardNumber distanceLogged = new LoggedDashboardNumber("DistanceShooting");
    //Decay = e^(-period/timeConstant) output = lastOutput*Decay + input * (1-Decay)
    LinearFilter filter = LinearFilter.singlePoleIIR(0.02, 0.02);
    private double[] inputBuffer;
    private double[] outputBuffer;

    public ChassisAimCommand(
            Swerve Swerve,
            Supplier<ShootingDecider.Destination> destinationSupplier,
            DoubleSupplier driverX,
            DoubleSupplier driverY) {
        this(Swerve, destinationSupplier, driverX, driverY, false);
    }

    public ChassisAimCommand(
            Swerve Swerve,
            Supplier<ShootingDecider.Destination> destinationSupplier,
            DoubleSupplier driverX,
            DoubleSupplier driverY,
            boolean isAuto) {
        this.swerve = Swerve;
        this.driverX = driverX;
        this.driverY = driverY;
        this.destinationSupplier = destinationSupplier;
        this.shootingDecider = ShootingDecider.getInstance();
        this.isAuto = isAuto;
    }

    @Override
    public void initialize() {
        boolean reset = false;
    }

    @Override
    public void execute() {
        if (!isAuto) {
            swerve.drive(
                    new Translation2d(
                            -driverX.getAsDouble() * Constants.SwerveConstants.maxSpeed.magnitude(),
                            -driverY.getAsDouble() * Constants.SwerveConstants.maxSpeed.magnitude()),
                    0,
                    true,
                    false);
        } else {
            swerve.autoDrive(new Translation2d(0, 0), 0, true, false);
        }

        ShootingParameters parameter = shootingDecider.getShootingParameter(
                destinationSupplier.get(),
                swerve.getLocalizer().getCoarseFieldPose(0));
        SmartDashboard.putNumber("Swerve/origin heading", parameter.getFieldAimingAngle().getDegrees());
        double degrees = parameter.getFieldAimingAngle().getDegrees();
        SmartDashboard.putNumber("Swerve/filtered heading", degrees);
        swerve.setHeadingTarget(degrees);
        distanceLogged.set(parameter.getDistance());
        swerve.setLockHeading(true);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
