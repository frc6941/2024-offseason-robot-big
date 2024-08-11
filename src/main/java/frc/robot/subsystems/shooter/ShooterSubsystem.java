package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

@Getter
public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine sysId;

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
        if (inputs.homed) return;
        if (inputs.armSupplyCurrent.magnitude() > armZeroCurrent.magnitude()) {
            io.setArmVoltage(Volts.zero());
            io.setArmHome(Radians.zero());
            getIo().setHomed(true);
            return;
        }
        io.setArmVoltage(armZeroVoltage);
    }

    public boolean aimingReady() {
        var velocityReady = Math.abs(inputs.leftShooterVelocity.magnitude() - inputs.targetShooterVelocity.magnitude()) < 6.28;//1
        var positionReady = Math.abs(inputs.armPosition.magnitude() - inputs.targetArmPosition.magnitude()) < 0.007 && Math.abs(inputs.armPosition.magnitude()) > 0.007;//0.007
        SmartDashboard.putBoolean("velocityReady", velocityReady);
        SmartDashboard.putBoolean("positionReady", positionReady);
        return velocityReady && positionReady;//TODO:fixme
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
