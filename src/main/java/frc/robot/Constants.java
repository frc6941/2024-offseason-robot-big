package frc.robot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.TunableNumber;
import org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static final boolean TUNING = true;
    public static final double LOOPER_DT = 1 / 100.0;

    public static Measure<Angle> armPosition = Radians.of(0);

    public static class AutoConstants {
        public static class swerveXGainsClass {
            public static final TunableNumber swerveX_KP = new TunableNumber("swerveX PID/kp", 0.3);
            public static final TunableNumber swerveX_KI = new TunableNumber("swerveX PID/ki", 0.);
            public static final TunableNumber swerveX_KD = new TunableNumber("swerveX PID/kd", 0.0001);
        }

        public static class swerveYGainsClass {
            public static final TunableNumber swerveY_KP = new TunableNumber("swerveY PID/kp", 0.3);
            public static final TunableNumber swerveY_KI = new TunableNumber("swerveY PID/ki", 0);
            public static final TunableNumber swerveY_KD = new TunableNumber("swerveY PID/kd", 0.0001);
        }

        public static class swerveOmegaGainsClass {
            public static final TunableNumber swerveOmega_KP = new TunableNumber("swerveOmega PID/kp", 0.3);
            public static final TunableNumber swerveOmega_KI = new TunableNumber("swerveOmega PID/ki", 0);
            public static final TunableNumber swerveOmega_KD = new TunableNumber("swerveOmega PID/kd", 0.0001);
        }
    }

    public static class IndexerConstants {
        public static final int INDEX_MOTOR_ID = 40;

        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);
        public final static Measure<Voltage> indexVoltage = Volts.of(5);
        public final static Measure<Voltage> trapIndexVoltage = Volts.of(2);
        public final static Measure<Voltage> indexShootVoltage = Volts.of(-16);

        public final static double indexRPM = 500;
        public final static double triggerRPM = 3500;


        public static class indexerGainsClass {
            public static final TunableNumber INDEXER_KP = new TunableNumber("INDEXER PID/kp", 0.25);
            public static final TunableNumber INDEXER_KI = new TunableNumber("INDEXER PID/ki", 0.0);
            public static final TunableNumber INDEXER_KD = new TunableNumber("INDEXER PID/kd", 0.001);
            public static final TunableNumber INDEXER_KA = new TunableNumber("INDEXER PID/ka", 0.0037512677);
            public static final TunableNumber INDEXER_KV = new TunableNumber("INDEXER PID/kv", 0.115);//0.107853495
            public static final TunableNumber INDEXER_KS = new TunableNumber("INDEXER PID/ks", 0.28475008);
        }
    }

    public static class IntakerConstants {
        public static final int INTAKE_MOTOR_ID = 30;

        public static final Measure<Voltage> intakeVoltage = Volts.of(-8.5);
    }

    public static class ShooterConstants {
        public static final int LEFT_SHOOTER_MOTOR_ID = 41;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 42;
        public static final int ARM_MOTOR_ID = 43;
        public static final int PULLER_MOTOR_ID = 44;

        // public static final TunableNumber custumAngle = new TunableNumber("custum
        // angle", 30);
        // public static final TunableNumber custumVoltage = new TunableNumber("custm
        // V", -7);

        // Shooter gains when deploying shooter to desired angle
        public static final Slot0Configs armGainsUp = new Slot0Configs()
                .withKP(400)
                .withKI(200)
                .withKD(15)
                // .withKV(0.12) // add 12v for desired velocity
                .withKS(0.25); // add 0.24v to overcome friction

        public static final Measure<Current> armZeroCurrent = Amps.of(1.0);
        public static final Measure<Voltage> armZeroVoltage = Volts.of(-2);

        public static final ClosedLoopRampsConfigs rampConfigs = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(0.3);

        public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(2.5)
                .withMotionMagicCruiseVelocity(1);

        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(90d / 24 * 90 / 24 * 84 / 14);
        public static final FeedbackConfigs pullerfeedbackConfigs = new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(8d / 64 * 16 / 64);

        public static final Measure<Voltage> shooterConstantVoltage = Volts.of(2);
        public static final double defaultShootRPM = -9.0 / 12 * 6380;
        public static final Measure<Voltage> ampShootingVoltage = Volts.of(-8);
        public static final double shortShootVoltage = -8.0 / 12 * 6380;
        public static final Measure<Distance> shortShootMaxDistance = Meters.of(2.7);
        public static final Measure<Distance> shootMaxDistance = Meters.of(3.7);
        public static final double farShootVoltage = -11.0 / 12 * 6380;
        public static final double pullVoltage = -8.0 / 12 * 6380;
        public final static Measure<Angle> ampDeployAngle = Degrees.of(165);
        public static final double shooterUpDownVoltage = -2.0 / 12 * 6380;
        public static final double shooterIndexVoltage = 13.0 / 12 * 6380;
        public static Measure<Angle> speakerArmOffsetNear = Degrees.of(17);
        public static Measure<Angle> speakerArmOffset = Degrees.of(44);//44
        public static Measure<Angle> speakerArmOffsetFar = Degrees.of(58.5);
        public static Measure<Angle> speakerArmOffsetMax = Degrees.of(64);

        public static class shooterGainsClass {
            public static final TunableNumber SHOOTER_KP = new TunableNumber("SHOOTER PID/kp", 0.3);
            public static final TunableNumber SHOOTER_KI = new TunableNumber("SHOOTER PID/ki", 0);
            public static final TunableNumber SHOOTER_KD = new TunableNumber("SHOOTER PID/kd", 0.005);
            public static final TunableNumber SHOOTER_KA = new TunableNumber("SHOOTER PID/ka", 0.0037512677);
            public static final TunableNumber SHOOTER_KV = new TunableNumber("SHOOTER PID/kv", 0.115);//0.107853495
            public static final TunableNumber SHOOTER_KS = new TunableNumber("SHOOTER PID/ks", 0.28475008);
        }
    }

    public static class BeamBreakConstants {
        public final static int INTAKER_BEAM_BREAK_ID = 1;
        public final static int INDEXER_BEAM_BREAK_ID = 2;
        public final static int SHOOTER_BEAM_BREAK_ID = 3;
    }

    public static class IndicatorConstants {
        public static final int LED_PORT = 0;
        public static final int LED_BUFFER_LENGTH = 17;
    }

    public static class VisionConstants {
        public static final String AIM_LIMELIGHT_NAME = "limelight";

        public static double REJECT_ANGULAR_SPEED = 360;//degree
        public static double REJECT_LINEAR_SPEED = 2.5;// m/s
    }

    public static class Logger {
        public static final boolean ENABLE_DEBUG = true;

        public static void debug(String... texts) {
            if (ENABLE_DEBUG) {
                System.out.println(String.join(" ", texts));
            }
        }
    }

    public class RobotConstants {

        public static final CommandXboxController driverController = new CommandXboxController(0);
        public static final CommandXboxController operatorController = new CommandXboxController(1);
        public static String CAN_BUS_NAME = "6941CANivore1";

    }

    public class SwerveDrivetrain {

        public static final TunableNumber LongShotAngle = new TunableNumber("Long shot angle", 25);

        public static final Measure<Voltage> MAX_VOLTAGE = Volts.of(12.0);

        public static final double VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.0003;// 0.0002

        public static final int PIGEON_ID = 1;

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANbusName(RobotConstants.CAN_BUS_NAME)
                .withPigeon2Id(PIGEON_ID)
                .withPigeon2Configs(null); // optional

        /**
         * The max speed of the swerve (should not larger than speedAt12Volts)
         */
        public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(4.5);
        /**
         * The max turning speed of the swerve
         */
        public static final Measure<Velocity<Angle>> maxAngularRate = RotationsPerSecond.of(1.5 * Math.PI);

        public static final double deadband = maxSpeed.magnitude() * 0.07;
        public static final double rotationalDeadband = maxAngularRate.magnitude() * 0.07;

        public static final SlewRateLimiter xLimiter = new SlewRateLimiter(3, -3.25, 0);
        public static final SlewRateLimiter yLimiter = new SlewRateLimiter(3, -3.25, 0);

        /**
         * Gearing between the drive motor output shaft and the wheel.
         */
        public static final double DRIVE_GEAR_RATIO = 6.7460317460317460317460317460317;
        /**
         * Theoretical free speed (m/s) at 12v applied output;
         */
        public static final Measure<Velocity<Distance>> speedAt12Volts = MetersPerSecond.of(6.0);
        public static final KinematicLimits DRIVETRAIN_UNCAPPED = new KinematicLimits(
                maxSpeed.magnitude(),
                50.0,
                5000);// maxAngularRate.magnitude()
        public static final SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(
                0.69522, 2.3623, 0.19367);
        /**
         * Spin PID
         */
        public static final Slot0Configs headingGains = new Slot0Configs()
                .withKP(0.04)
                .withKI(0)
                .withKD(0);
        /**
         * Gearing between the steer motor output shaft and the azimuth gear.
         */
        private static final double STEER_GEAR_RATIO = 21.428571428571428571428571428571;
        /**
         * Radius of the wheel in meters.
         */
        private static final Measure<Distance> wheelRadius = Inches.of(2);
        /**
         * Circumference of the wheel in meters.
         */
        public static final Measure<Distance> wheelCircumferenceMeters = Meters
                .of(wheelRadius.magnitude() * 2 * Math.PI);
        /**
         * The stator current at which the wheels start to slip
         */
        private static final Measure<Current> slipCurrent = Amps.of(90.0);
        /**
         * Swerve steering gains
         */
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(120)// 120
                .withKI(0.2)// 0.2
                .withKD(0.005)// 0.005
                .withKS(0)
                .withKV(0)
                .withKA(0);
        /**
         * Swerve driving gains
         */
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(1)// 0.3
                .withKI(0)
                .withKD(0)
                .withKS(0)
                .withKV(0.12)// 2//0.00001
                .withKA(0);
        /**
         * The closed-loop output type to use for the steer motors;
         * This affects the PID/FF gains for the steer motors
         */
        private static final SwerveModule.ClosedLoopOutputType steerClosedLoopOutput = SwerveModule.ClosedLoopOutputType.Voltage;
        /**
         * The closed-loop output type to use for the drive motors;
         * This affects the PID/FF gains for the drive motors
         */
        private static final SwerveModule.ClosedLoopOutputType driveClosedLoopOutput = SwerveModule.ClosedLoopOutputType.Voltage;
        /**
         * Simulation only
         */
        private static final double STEER_INERTIA = 0.00001;
        /**
         * Simulation only
         */
        private static final double DRIVE_INERTIA = 0.001;
        /**
         * Simulation only
         */
        private static final Measure<Voltage> steerFrictionVoltage = Volts.of(0.25);
        /**
         * Simulation only
         */
        private static final Measure<Voltage> driveFrictionVoltage = Volts.of(0.25);

        /**
         * Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
         */
        private static final double COUPLE_RATIO = 3.5;

        private static final boolean STEER_MOTOR_REVERSED = true;

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withWheelRadius(wheelRadius.in(Inches))
                .withSlipCurrent(slipCurrent.magnitude())
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSpeedAt12VoltsMps(speedAt12Volts.magnitude())
                .withSteerInertia(STEER_INERTIA)
                .withDriveInertia(DRIVE_INERTIA)
                .withSteerFrictionVoltage(steerFrictionVoltage.magnitude())
                .withDriveFrictionVoltage(driveFrictionVoltage.magnitude())
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withSteerMotorInverted(STEER_MOTOR_REVERSED);

        // Front Left
        private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 15;
        private static final int FRONT_LEFT_STEER_MOTOR_ID = 14;
        private static final int FRONT_LEFT_ENCODER_ID = 20;
        private static final double FRONT_LEFT_ENCODER_OFFSET = 0.4421621094;// 0.052955;//0.127686//0.5329550781
        private static final Measure<Distance> frontLeftXPos = Meters.of(0.5);
        private static final Measure<Distance> frontLeftYPos = Meters.of(0.5);
        public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_ENCODER_ID,
                FRONT_LEFT_ENCODER_OFFSET,
                frontLeftXPos.magnitude(),
                frontLeftYPos.magnitude(),
                false);
        // Front Right
        private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
        private static final int FRONT_RIGHT_STEER_MOTOR_ID = 7;
        private static final int FRONT_RIGHT_ENCODER_ID = 21;
        private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.1270234375;// 0.125685;//0.13623046875//0.117686//0.046875
        private static final Measure<Distance> frontRightXPos = Meters.of(0.5);
        private static final Measure<Distance> frontRightYPos = Meters.of(-0.5);  
        public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_ENCODER_ID,
                FRONT_RIGHT_ENCODER_OFFSET,
                frontRightXPos.magnitude(),
                frontRightYPos.magnitude(),
                true);
        // Back Left
        private static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
        private static final int BACK_LEFT_STEER_MOTOR_ID = 3;
        private static final int BACK_LEFT_ENCODER_ID = 9;
        private static final double BACK_LEFT_ENCODER_OFFSET = 0.7644375;// 0.773925;//-0.223//0.401611//0.77392578125
        private static final Measure<Distance> backLeftXPos = Meters.of(-0.5);
        private static final Measure<Distance> backLeftYPos = Meters.of(0.5);
        public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_ENCODER_ID,
                BACK_LEFT_ENCODER_OFFSET,
                backLeftXPos.magnitude(),
                backLeftYPos.magnitude(),
                false);
        // Back Right
        private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 10;
        private static final int BACK_RIGHT_STEER_MOTOR_ID = 6;
        private static final int BACK_RIGHT_ENCODER_ID = 12;
        private static final double BACK_RIGHT_ENCODER_OFFSET = 0.4265703125;// 0.422119;//-0.5684550781//-0.064453//0.432279296875
        private static final Measure<Distance> backRightXPos = Meters.of(-0.5);
        private static final Measure<Distance> backRightYPos = Meters.of(-0.5);
        public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_ENCODER_ID,
                BACK_RIGHT_ENCODER_OFFSET,
                backRightXPos.magnitude(),
                backRightYPos.magnitude(),
                true);

        public static SwerveModuleConstants[] modules = {FrontLeft, FrontRight, BackLeft, BackRight};

        public static final Translation2d[] modulePlacements = new Translation2d[]{
                new Translation2d(SwerveDrivetrain.FrontLeft.LocationX,
                        SwerveDrivetrain.FrontLeft.LocationY),
                new Translation2d(SwerveDrivetrain.FrontRight.LocationX,
                        SwerveDrivetrain.FrontRight.LocationY),
                new Translation2d(SwerveDrivetrain.BackLeft.LocationX,
                        SwerveDrivetrain.BackLeft.LocationY),
                new Translation2d(SwerveDrivetrain.BackRight.LocationX,
                        SwerveDrivetrain.BackRight.LocationY)
        };

        public static class steerGainsClass {
            public static final TunableNumber STEER_KP = new TunableNumber("STEER PID/kp", 120);
            public static final TunableNumber STEER_KI = new TunableNumber("STEER PID/ki", 0.2);
            public static final TunableNumber STEER_KD = new TunableNumber("STEER PID/kd", 0.005);
            public static final TunableNumber STEER_KA = new TunableNumber("STEER PID/ka", 0);
            public static final TunableNumber STEER_KV = new TunableNumber("STEER PID/kv", 0);
            public static final TunableNumber STEER_KS = new TunableNumber("STEER PID/ks", 0);
        }
        // public static final KinematicLimits DRIVETRAIN_SMOOTHED = new
        // KinematicLimits(
        // 4.5,
        // 30.0,
        // 200.0);
        // public static final KinematicLimits DRIVETRAIN_LIMITED = new KinematicLimits(
        // 2.0,
        // 10.0,
        // 1200.0);
        // public static final KinematicLimits DRIVETRAIN_ROBOT_ORIENTED = new
        // KinematicLimits(
        // 2.0,
        // 5.0,
        // 1500.0);

        public static class driveGainsClass {
            public static final TunableNumber DRIVE_KP = new TunableNumber("DRIVE PID/kp", 0.3);
            public static final TunableNumber DRIVE_KI = new TunableNumber("DRIVE PID/ki", 0);
            public static final TunableNumber DRIVE_KD = new TunableNumber("DRIVE PID/kd", 0.0001);
            public static final TunableNumber DRIVE_KA = new TunableNumber("DRIVE PID/ka", 0);
            public static final TunableNumber DRIVE_KV = new TunableNumber("DRIVE PID/kv", 0.12);
            public static final TunableNumber DRIVE_KS = new TunableNumber("DRIVE PID/ks", 0.14);
        }

        public static class headingController {
            public static final TunableNumber HEADING_KP = new TunableNumber("HEADING PID/kp", 0.08);
            public static final TunableNumber HEADING_KI = new TunableNumber("HEADING PID/ki", 0.0002);
            public static final TunableNumber HEADING_KD = new TunableNumber("HEADING PID/kd", 0.002);
            //TODO:Fixme
        }
    }

    public class ArmConstants {
        public static final LoggedDashboardNumber ArmP = new LoggedDashboardNumber("arm p", 400);
        public static final LoggedDashboardNumber ArmI = new LoggedDashboardNumber("arm i", 200);
        public static final LoggedDashboardNumber ArmD = new LoggedDashboardNumber("arm d", 15);
    }

}
