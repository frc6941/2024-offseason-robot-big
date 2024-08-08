package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.TunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class PreShootTestCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private double RPM;
    private TunableNumber flyWheelVelocity = new TunableNumber("flyWheelVelocity", 0.0);
    private LoggedDashboardNumber distanceLogged = new LoggedDashboardNumber("Distance");

    public PreShootTestCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        distanceLogged.set(Limelight.getInstance().getSpeakerRelativePosition().getNorm());
        shooterSubsystem.getIo().setFlyWheelVelocity(flyWheelVelocity.get());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setFlyWheelDirectVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }
}
