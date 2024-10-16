package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.display.OperatorDashboard;
import frc.robot.utils.shooting.ShootingDecider;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.shooterConstantVoltage;

@Getter
public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    private final ShootingDecider.Destination destinationSupplier = ShootingDecider.Destination.AMP;
    private final ShootingDecider shootingDecider = ShootingDecider.getInstance();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        this.io.setFlyWheelDirectVoltage(shooterConstantVoltage);

        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Flywheel/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism((voltage) -> io.runVolts(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
//        if (OperatorDashboard.getInstance().getFlyWheelOn().get().getBoolean()) {
//                this.getIo().setFlyWheelVelocity(
//                        shootingDecider.getShootingParameter(
//                                destinationSupplier,
//                                Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)
//                        ).getShootingVelocity());
    }

    public boolean ShooterVelocityReady() {
        var velocityReady = Math.abs(inputs.leftShooterVelocity.magnitude() - inputs.targetShooterVelocity.magnitude()) < 12.56;//1
        SmartDashboard.putBoolean("velocityReady", velocityReady);
        OperatorDashboard.getInstance().updateShooterReady(velocityReady);
        return velocityReady;
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

}
