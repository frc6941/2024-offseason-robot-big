package frc.robot.commands.auto;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beambreak.BeamBreakSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.utils.ShootingParameters.ShootingParameters;
import frc.robot.utils.ShootingParameters.SpeakerParameterTable;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class PreArmAutoCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final BeamBreakSubsystem beamBreakSubsystem;
    LinearFilter filter = LinearFilter.singlePoleIIR(0.2, 0.02);
    private LoggedDashboardNumber distanceLogged = new LoggedDashboardNumber("Distance");

    public PreArmAutoCommand(
            ShooterSubsystem shooterSubsystem,
            IndicatorSubsystem indicatorSubsystem,
            BeamBreakSubsystem beamBreakSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.beamBreakSubsystem = beamBreakSubsystem;
    }

    @Override
    public void execute() {
        if (!beamBreakSubsystem.isIntakeReady()) {
            shooterSubsystem.getIo().setArmPosition(Radians.zero(), false);
            return;
        }
        this.indicatorSubsystem.setPattern(IndicatorIO.Patterns.AIMED);

        var distance = Limelight.getInstance().getSpeakerRelativePosition().getNorm();

        ShootingParameters parameter = SpeakerParameterTable.getInstance().getParameters(distance);

        if (Math.abs(
                parameter.getAngle() -
                        shooterSubsystem.getInputs().armPosition.in(Degrees)) >= 0.5) {
            shooterSubsystem
                    .getIo()
                    .setArmPosition(
                            Radians.of(
                                    Units.degreesToRadians(parameter.getAngle())));
        }
        distanceLogged.set(distance);
        filter.calculate(Limelight.getInstance().getSpeakerRelativePosition().getAngle().getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
