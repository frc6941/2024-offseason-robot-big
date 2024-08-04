package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.TunableNumber;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

@Getter
public class ShooterSubsystem extends SubsystemBase {
    public static final TunableNumber DRIVE_KA = new TunableNumber("SHOOTER PID/ka", 0.0011679);
    public static final TunableNumber DRIVE_KV = new TunableNumber("SHOOTER PID/kv", 0.018764);
    public static final TunableNumber DRIVE_KS = new TunableNumber("SHOOTER PID/ks", 0.16172);
    public static final TunableNumber DRIVE_KP = new TunableNumber("SHOOTER PID/kp", 0.0076849);
    public static final TunableNumber DRIVE_KI = new TunableNumber("SHOOTER PID/ki", 0);
    public static final TunableNumber DRIVE_KD = new TunableNumber("SHOOTER PID/kd", 0);
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine sysId;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        this.io.setFlyWheelVoltage(shooterConstantVoltage);

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
