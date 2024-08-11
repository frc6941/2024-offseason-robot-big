package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ArmConstants.armZeroCurrent;
import static frc.robot.Constants.ArmConstants.armZeroVoltage;

@Getter
public class ArmSubsystem extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public ArmSubsystem(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        if (inputs.homed) return;
        if (inputs.armSupplyCurrent.magnitude() > armZeroCurrent.magnitude()) {
            io.setArmVoltage(Volts.zero());
            io.setArmHome(Radians.zero());
            getIo().setHomed(true);
            return;
        }
        io.setArmVoltage(armZeroVoltage);
    }

    public boolean armAimingReady() {
        var positionReady = Math.abs(inputs.armPosition.magnitude() - inputs.targetArmPosition.magnitude()) < 0.007 && Math.abs(inputs.armPosition.magnitude()) > 0.007;
        SmartDashboard.putBoolean("positionReady", positionReady);
        return positionReady;//TODO:fixme
    }
}
