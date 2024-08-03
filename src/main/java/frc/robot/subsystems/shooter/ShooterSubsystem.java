package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ShooterConstants.*;

@Getter
public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();


    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        this.io.setFlyWheelVoltage(shooterConstantVoltage);
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
}
