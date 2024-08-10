package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;
import frc.robot.utils.shooting.ShootingParameters;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.team254.lib.geometry.Pose2d;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class ArmAimCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final ShootingDecider shootingDecider;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;

    public ArmAimCommand(
            ShooterSubsystem shooterSubsystem,
            Supplier<ShootingDecider.Destination> destinationSupplier
    ) {
        this.shootingDecider = ShootingDecider.getInstance();
        this.shooterSubsystem = shooterSubsystem;
        this.destinationSupplier = destinationSupplier;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {        



        shooterSubsystem.getIo().setArmPosition(Radians.of(
            Units.degreesToRadians(
                shootingDecider.getShootingParameter(
                    destinationSupplier.get(), 
                    Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)
                ).getShootingAngle())));
    }
    

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo().setArmPosition(Radians.zero());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

