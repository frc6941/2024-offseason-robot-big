package org.frcteam6941.utils;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.team254.lib.drivers.LazyTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class CTREFactory {

    private final static int kTimeoutMs = 100;

    public static class Configuration {
        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        // factory default
        public double NEUTRAL_DEADBAND = 0.04;

        public boolean ENABLE_CURRENT_LIMIT = false;
        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int FORWARD_SOFT_LIMIT = 0;
        public int REVERSE_SOFT_LIMIT = 0;

        public boolean INVERTED = false;
        public boolean SENSOR_PHASE = false;

        public int CONTROL_FRAME_PERIOD_MS = 20; // 10
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 20; // 10
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
        
        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_20Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(false, 300, 700, 1);
        public SupplyCurrentLimitConfiguration SUPPLY_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(false, 40, 100, 1);

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kSlaveConfiguration = new Configuration();

    static {
        // This control frame value seems to need to be something reasonable to avoid the Talon's
        // LEDs behaving erratically. Potentially try to increase as much as possible.
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 100;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ENABLE_SOFT_LIMIT = false;
    }

    public static TalonFX createDefaultTalonFX(int id, boolean isOnCanivore) {
        return createTalonFX(id, isOnCanivore, kDefaultConfiguration);
    }

    public static TalonSRX createDefaultTalonSRX(int id) {
        return createTalonSRX(id, kDefaultConfiguration);
    }

    public static VictorSPX createDefaultVictorSPX(int id) {
        return createVictorSPX(id, kDefaultConfiguration);
    }

    public static TalonFX createPermanentSlaveTalon(int id, int master_id, boolean isOnCanivore) {
        final TalonFX talon = createTalonFX(id, isOnCanivore, kSlaveConfiguration);
        talon.set(ControlMode.Follower, master_id);
        return talon;
    }

    public static TalonFX createTalonFX(int id, boolean isOnCanivore, Configuration config) {
        TalonFX talon = new TalonFX(id, isOnCanivore ? "canivore" : "");
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.clearMotionProfileHasUnderrun(kTimeoutMs);
        talon.clearMotionProfileTrajectories();

        talon.clearStickyFaults(kTimeoutMs);

        talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.Disabled, kTimeoutMs);
        talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.Disabled, kTimeoutMs);
        talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

        // Turn off re-zeroing by default.
        talon.configSetParameter(
                ParamEnum.eClearPositionOnLimitF, 0, 0, 0, kTimeoutMs);
        talon.configSetParameter(
                ParamEnum.eClearPositionOnLimitR, 0, 0, 0, kTimeoutMs);

        talon.configNominalOutputForward(0, kTimeoutMs);
        talon.configNominalOutputReverse(0, kTimeoutMs);
        talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, kTimeoutMs);

        talon.configPeakOutputForward(1.0, kTimeoutMs);
        talon.configPeakOutputReverse(-1.0, kTimeoutMs);

        talon.setNeutralMode(config.NEUTRAL_MODE);

        talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, kTimeoutMs);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);

        talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, kTimeoutMs);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);
        talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        talon.setInverted(config.INVERTED);
        talon.setSensorPhase(config.SENSOR_PHASE);

        talon.selectProfileSlot(0, 0);

        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW,
                kTimeoutMs);

        talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, kTimeoutMs);
        talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, kTimeoutMs);

        talon.configVoltageCompSaturation(0.0, kTimeoutMs);
        talon.configVoltageMeasurementFilter(32, kTimeoutMs);
        talon.enableVoltageCompensation(false);

        talon.configStatorCurrentLimit(config.STATOR_CURRENT_LIMIT);
        talon.configSupplyCurrentLimit(config.SUPPLY_CURRENT_LIMIT);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
                config.GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                config.FEEDBACK_STATUS_FRAME_RATE_MS, kTimeoutMs);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
                config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
                config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
                config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, kTimeoutMs);

        talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

        return talon;
    }

    public static TalonSRX createTalonSRX(int id, Configuration config) {
        TalonSRX talon = new LazyTalonSRX(id);
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.clearMotionProfileHasUnderrun(kTimeoutMs);
        talon.clearMotionProfileTrajectories();

        talon.clearStickyFaults(kTimeoutMs);

        talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.Disabled, kTimeoutMs);
        talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.Disabled, kTimeoutMs);
        talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

        // Turn off re-zeroing by default.
        talon.configSetParameter(
                ParamEnum.eClearPositionOnLimitF, 0, 0, 0, kTimeoutMs);
        talon.configSetParameter(
                ParamEnum.eClearPositionOnLimitR, 0, 0, 0, kTimeoutMs);

        talon.configNominalOutputForward(0, kTimeoutMs);
        talon.configNominalOutputReverse(0, kTimeoutMs);
        talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, kTimeoutMs);

        talon.configPeakOutputForward(1.0, kTimeoutMs);
        talon.configPeakOutputReverse(-1.0, kTimeoutMs);

        talon.setNeutralMode(config.NEUTRAL_MODE);

        talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, kTimeoutMs);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);

        talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, kTimeoutMs);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);
        talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        talon.setInverted(config.INVERTED);
        talon.setSensorPhase(config.SENSOR_PHASE);

        talon.selectProfileSlot(0, 0);

        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW,
                kTimeoutMs);

        talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, kTimeoutMs);
        talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, kTimeoutMs);

        talon.configVoltageCompSaturation(0.0, kTimeoutMs);
        talon.configVoltageMeasurementFilter(32, kTimeoutMs);
        talon.enableVoltageCompensation(false);

        talon.configSupplyCurrentLimit(config.SUPPLY_CURRENT_LIMIT);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
                config.GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                config.FEEDBACK_STATUS_FRAME_RATE_MS, kTimeoutMs);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
                config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
                config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, kTimeoutMs);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
                config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, kTimeoutMs);

        talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

        return talon;
    }

    public static VictorSPX createVictorSPX(int id, Configuration config) {
        VictorSPX talon = new LazyVictorSPX(id);
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.clearMotionProfileHasUnderrun(kTimeoutMs);
        talon.clearMotionProfileTrajectories();

        talon.clearStickyFaults(kTimeoutMs);
        talon.overrideLimitSwitchesEnable(config.ENABLE_LIMIT_SWITCH);

        // Turn off re-zeroing by default.
        talon.configSetParameter(
                ParamEnum.eClearPositionOnLimitF, 0, 0, 0, kTimeoutMs);
        talon.configSetParameter(
                ParamEnum.eClearPositionOnLimitR, 0, 0, 0, kTimeoutMs);

        talon.configNominalOutputForward(0, kTimeoutMs);
        talon.configNominalOutputReverse(0, kTimeoutMs);
        talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, kTimeoutMs);

        talon.configPeakOutputForward(1.0, kTimeoutMs);
        talon.configPeakOutputReverse(-1.0, kTimeoutMs);

        talon.setNeutralMode(config.NEUTRAL_MODE);

        talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, kTimeoutMs);
        talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);

        talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, kTimeoutMs);
        talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, kTimeoutMs);
        talon.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        talon.setInverted(config.INVERTED);
        talon.setSensorPhase(config.SENSOR_PHASE);

        talon.selectProfileSlot(0, 0);

        talon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs);
        talon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW,
                kTimeoutMs);

        talon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, kTimeoutMs);
        talon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, kTimeoutMs);

        talon.configVoltageCompSaturation(0.0, kTimeoutMs);
        talon.configVoltageMeasurementFilter(32, kTimeoutMs);
        talon.enableVoltageCompensation(false);
        talon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

        return talon;
    }
}
