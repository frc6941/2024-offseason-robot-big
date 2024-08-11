package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.swerve.Swerve;

import java.util.Optional;

public class ChoreoDriveTestCommand extends Command {

    ChoreoTrajectory traj = Choreo.getTrajectory("NewPath");
    IndicatorSubsystem indicatorSubsystem;
    private Swerve swerve;

    ChoreoDriveTestCommand(Swerve swerve, IndicatorSubsystem indicatorSubsystem) {
        this.swerve = swerve;
        this.indicatorSubsystem = indicatorSubsystem;
    }

    @Override
    public void execute() {
        Choreo.choreoSwerveCommand(
                traj,
                () -> swerve.getLocalizer().getLatestPose(),
                new PIDController(
                        Constants.AutoConstants.swerveXGainsClass.swerveX_KP.get(),
                        Constants.AutoConstants.swerveXGainsClass.swerveX_KI.get(),
                        Constants.AutoConstants.swerveXGainsClass.swerveX_KD.get()),
                new PIDController(
                        Constants.AutoConstants.swerveYGainsClass.swerveY_KP.get(),
                        Constants.AutoConstants.swerveYGainsClass.swerveY_KI.get(),
                        Constants.AutoConstants.swerveYGainsClass.swerveY_KD.get()),
                new PIDController(
                        Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KP.get(),
                        Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KI.get(),
                        Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KD.get()),
                (ChassisSpeeds speeds) ->
                        swerve.autoDrive(
//                                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
//                                speeds.omegaRadiansPerSecond,
//                                false,
//                                false),
                                new Translation2d(0, 1),
                                0,
                                false,
                                false),
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                });
    }
}
