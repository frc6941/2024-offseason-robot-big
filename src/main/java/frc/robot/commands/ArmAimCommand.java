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
import frc.robot.utils.ShootingParameters.ShootingDecider;
import frc.robot.utils.ShootingParameters.ShootingParameters;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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

        var distance = Limelight.getInstance().getSpeakerRelativePosition().getNorm();

        shooterSubsystem.getIo().setArmPosition(Radians.of(Units.degreesToRadians(parameter.getAngle())));
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

