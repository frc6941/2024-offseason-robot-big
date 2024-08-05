package frc.robot.commands.test;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.ShootingParameters;
import frc.robot.utils.ShootingParametersTable;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.shortShootVoltage;

import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import frc.robot.utils.TunableNumber;

public class PreShootTestCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private Measure<Voltage> defaultVoltage = shortShootVoltage;
	private TunableNumber flyWheelVoltage = new TunableNumber("flyWheelVoltage", 0.0);
	private LoggedDashboardNumber distanceLogged = new LoggedDashboardNumber("Distance");
	
    public PreShootTestCommand(ShooterSubsystem shooterSubsystem) {
		this.shooterSubsystem = shooterSubsystem;
		addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
		defaultVoltage = shortShootVoltage;
    }

    @Override
	public void execute() {
		distanceLogged.set(Limelight.getInstance().getSpeakerRelativePosition().getNorm());
        shooterSubsystem.getIo().setFlyWheelVoltage(Volts.of(flyWheelVoltage.get()));
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.getIo()
                .setFlyWheelVoltage(Constants.ShooterConstants.shooterConstantVoltage);
    }
}
