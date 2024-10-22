package frc.robot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.TunableNumber;
import org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits;

import static edu.wpi.first.units.Units.*;

public class Constants {
    public static final boolean TUNING = false;
    public static final double LOOPER_DT = 1 / 50.0;

    public static Measure<Angle> armPosition = Radians.of(0);

    public static class AutoConstants {
        public static class swerveXGainsClass {
            public static final TunableNumber swerveX_KP = new TunableNumber("swerveX PID/kp", 5);//1.4
            public static final TunableNumber swerveX_KI = new TunableNumber("swerveX PID/ki", 0);
            public static final TunableNumber swerveX_KD = new TunableNumber("swerveX PID/kd", 0.0);
        }

        public static class swerveYGainsClass {
            public static final TunableNumber swerveY_KP = new TunableNumber("swerveY PID/kp", 0.0);
            public static final TunableNumber swerveY_KI = new TunableNumber("swerveY PID/ki", 0);
            public static final TunableNumber swerveY_KD = new TunableNumber("swerveY PID/kd", 0);
        }

        public static class swerveOmegaGainsClass {
            public static final TunableNumber swerveOmega_KP = new TunableNumber("swerveOmega PID/kp", 3);//0.7
            public static final TunableNumber swerveOmega_KI = new TunableNumber("swerveOmega PID/ki", 0);
            public static final TunableNumber swerveOmega_KD = new TunableNumber("swerveOmega PID/kd", 0);
        }
    }

    public static class IndexerConstants {
        public static final int INDEX_MOTOR_ID = 40;

        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);

        public final static double indexRPM = 1000;
        public final static double triggerRPM = 4500;

        public static class indexerGainsClass {
            public static final TunableNumber INDEXER_KP = new TunableNumber("INDEXER PID/kp", 0.25);
            public static final TunableNumber INDEXER_KI = new TunableNumber("INDEXER PID/ki", 0.0);
            public static final TunableNumber INDEXER_KD = new TunableNumber("INDEXER PID/kd", 0.001);
            public static final TunableNumber INDEXER_KA = new TunableNumber("INDEXER PID/ka", 0.0037512677);
            public static final TunableNumber INDEXER_KV = new TunableNumber("INDEXER PID/kv", 0.115);// 0.107853495
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

        public static final Measure<Voltage> shooterConstantVoltage = Volts.of(-4);
        public static final TunableNumber skewValue = new TunableNumber("Shooter skew", 0);
        public static double kToFFactor = 0.2;
        public static boolean useSmartDashboardForSkew = false;
        public static boolean useShootOnMove = false;

        public static class shooterGainsClass {
            public static final TunableNumber SHOOTER_KP = new TunableNumber("SHOOTER PID/kp", 0.2);
            public static final TunableNumber SHOOTER_KI = new TunableNumber("SHOOTER PID/ki", 0);
            public static final TunableNumber SHOOTER_KD = new TunableNumber("SHOOTER PID/kd", 0.001);
            public static final TunableNumber SHOOTER_KA = new TunableNumber("SHOOTER PID/ka", 0.0037512677);
            public static final TunableNumber SHOOTER_KV = new TunableNumber("SHOOTER PID/kv", 0.113);// 0.107853495
            public static final TunableNumber SHOOTER_KS = new TunableNumber("SHOOTER PID/ks", 0.28475008);
        }
    }

    public static class ArmConstants {
        public static final int ARM_MOTOR_ID = 43;
        public static final int PULLER_MOTOR_ID = 44;
        public static final Measure<Current> armZeroCurrent = Amps.of(1.0);
        public static final Measure<Voltage> armZeroVoltage = Volts.of(-1.5);
        public static final Measure<Voltage> armUpDownVoltage = Volts.of(-4);
        public static final Measure<Angle> armOutPosition = Degrees.of(25);
        public static final ClosedLoopRampsConfigs rampConfigs = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(0.3);
        public static final MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs()
                .withMotionMagicAcceleration(3)//?
                .withMotionMagicCruiseVelocity(2);//too low
        public static final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake);
        public static final FeedbackConfigs feedbackConfigs = new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(90d / 24 * 90 / 24 * 84 / 14);
        public static final FeedbackConfigs pullerfeedbackConfigs = new FeedbackConfigs()
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                .withSensorToMechanismRatio(8d / 64 * 16 / 64);
        public static final double pullVelocity = 4000;

        public static class armGainsClass {
            public static final TunableNumber ARM_KP = new TunableNumber("ARM PID/kp", 400);//todo change it 
            public static final TunableNumber ARM_KI = new TunableNumber("ARM PID/ki", 200);
            public static final TunableNumber ARM_KD = new TunableNumber("ARM PID/kd", 15);
            public static final TunableNumber ARM_KA = new TunableNumber("ARM PID/ka", 0);
            public static final TunableNumber ARM_KV = new TunableNumber("ARM PID/kv", 0);// 0.107853495
            public static final TunableNumber ARM_KS = new TunableNumber("ARM PID/ks", 0.25);
        }

        public static class pullerGainsClass {
            public static final TunableNumber PULLER_KP = new TunableNumber("PULLER PID/kp", 0.01);
            public static final TunableNumber PULLER_KI = new TunableNumber("PULLER PID/ki", 0);
            public static final TunableNumber PULLER_KD = new TunableNumber("PULLER PID/kd", 0);
            public static final TunableNumber PULLER_KA = new TunableNumber("PULLER PID/ka", 0.0037512677);
            public static final TunableNumber PULLER_KV = new TunableNumber("PULLER PID/kv", 0.113);// 0.107853495
            public static final TunableNumber PULLER_KS = new TunableNumber("PULLER PID/ks", 0.28475008);
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

        public static double REJECT_ANGULAR_SPEED = 360;// degree
        public static double REJECT_LINEAR_SPEED = 2.5;// m/s
    }

    @SuppressWarnings("unused")
    public static class FieldConstants {
        public static final double fieldLength = edu.wpi.first.math.util.Units.inchesToMeters(651.223);
        public static final double fieldWidth = edu.wpi.first.math.util.Units.inchesToMeters(323.277);
        public static final double wingX = edu.wpi.first.math.util.Units.inchesToMeters(229.201);
        public static final double wingOpponentX = fieldLength - wingX;
        public static final double podiumX = edu.wpi.first.math.util.Units.inchesToMeters(126.75);
        public static final double startingLineX = edu.wpi.first.math.util.Units.inchesToMeters(74.111);

        public static final Translation2d ampCenter = new Translation2d(
                edu.wpi.first.math.util.Units.inchesToMeters(72.455), fieldWidth);
        public static final double aprilTagWidth = edu.wpi.first.math.util.Units.inchesToMeters(6.50);

        /**
         * Staging locations for each note
         */
        public static final class StagingLocations {
            public static final double centerlineX = fieldLength / 2.0;

            // need to update
            public static final double centerlineFirstY = edu.wpi.first.math.util.Units.inchesToMeters(29.638);
            public static final double centerlineSeparationY = edu.wpi.first.math.util.Units.inchesToMeters(66);
            public static final double spikeX = edu.wpi.first.math.util.Units.inchesToMeters(114);
            // need
            public static final double spikeFirstY = edu.wpi.first.math.util.Units.inchesToMeters(161.638);
            public static final double spikeSeparationY = edu.wpi.first.math.util.Units.inchesToMeters(57);

            public static final Translation2d[] centerlineTranslations = new Translation2d[5];
            public static final Translation2d[] spikeTranslations = new Translation2d[3];

            static {
                for (int i = 0; i < centerlineTranslations.length; i++) {
                    centerlineTranslations[i] = new Translation2d(centerlineX,
                            centerlineFirstY + (i * centerlineSeparationY));
                }
            }

            static {
                for (int i = 0; i < spikeTranslations.length; i++) {
                    spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
                }
            }
        }

        /**
         * Each corner of the speaker *
         */
        public static final class Speaker {

            // corners (blue alliance origin)
            public static final Translation3d topRightSpeaker = new Translation3d(
                    edu.wpi.first.math.util.Units.inchesToMeters(18.055),
                    edu.wpi.first.math.util.Units.inchesToMeters(238.815),
                    edu.wpi.first.math.util.Units.inchesToMeters(83.091));

            public static final Translation3d topLeftSpeaker = new Translation3d(
                    edu.wpi.first.math.util.Units.inchesToMeters(18.055),
                    edu.wpi.first.math.util.Units.inchesToMeters(197.765),
                    edu.wpi.first.math.util.Units.inchesToMeters(83.091));

            public static final Translation3d bottomRightSpeaker = new Translation3d(0.0,
                    edu.wpi.first.math.util.Units.inchesToMeters(238.815),
                    edu.wpi.first.math.util.Units.inchesToMeters(78.324));
            public static final Translation3d bottomLeftSpeaker = new Translation3d(0.0,
                    edu.wpi.first.math.util.Units.inchesToMeters(197.765),
                    edu.wpi.first.math.util.Units.inchesToMeters(78.324));

            /**
             * Center of the speaker opening (blue alliance)
             */
            public static final Translation3d centerSpeakerOpening = bottomLeftSpeaker.interpolate(topRightSpeaker,
                    0.5);
        }

        public static final class Subwoofer {
            public static final Pose2d ampFaceCorner = new Pose2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(35.775),
                    edu.wpi.first.math.util.Units.inchesToMeters(239.366),
                    Rotation2d.fromDegrees(-120));

            public static final Pose2d sourceFaceCorner = new Pose2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(35.775),
                    edu.wpi.first.math.util.Units.inchesToMeters(197.466),
                    Rotation2d.fromDegrees(120));

            public static final Pose2d centerFace = new Pose2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(35.775),
                    edu.wpi.first.math.util.Units.inchesToMeters(218.416),
                    Rotation2d.fromDegrees(180));
        }

        public static final class Stage {
            public static final Pose2d center = new Pose2d(edu.wpi.first.math.util.Units.inchesToMeters(192.55),
                    edu.wpi.first.math.util.Units.inchesToMeters(161.638), new Rotation2d());
            public static final Pose2d podiumLeg = new Pose2d(edu.wpi.first.math.util.Units.inchesToMeters(126.75),
                    edu.wpi.first.math.util.Units.inchesToMeters(161.638), new Rotation2d());
            public static final Pose2d ampLeg = new Pose2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(220.873),
                    edu.wpi.first.math.util.Units.inchesToMeters(212.425),
                    Rotation2d.fromDegrees(-30));
            public static final Pose2d sourceLeg = new Pose2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(220.873),
                    edu.wpi.first.math.util.Units.inchesToMeters(110.837),
                    Rotation2d.fromDegrees(30));

            public static final Pose2d centerPodiumAmpChain = new Pose2d(
                    podiumLeg.getTranslation().interpolate(ampLeg.getTranslation(), 0.5),
                    Rotation2d.fromDegrees(120.0));
            public static final double centerToChainDistance = center.getTranslation()
                    .getDistance(centerPodiumAmpChain.getTranslation());
            public static final Pose2d centerAmpSourceChain = new Pose2d(
                    ampLeg.getTranslation().interpolate(sourceLeg.getTranslation(), 0.5), new Rotation2d());
            public static final Pose2d centerSourcePodiumChain = new Pose2d(
                    sourceLeg.getTranslation().interpolate(podiumLeg.getTranslation(), 0.5),
                    Rotation2d.fromDegrees(240.0));
        }

        public static final class Amp {
            public static final Translation2d ampTapeTopCorner = new Translation2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(130.0),
                    edu.wpi.first.math.util.Units.inchesToMeters(305.256));
            public static final double ampBottomY = fieldWidth - edu.wpi.first.math.util.Units.inchesToMeters(17.75);
        }
    }

    public static class RobotConstants {

        public static final CommandXboxController driverController = new CommandXboxController(0);
        public static final CommandXboxController operatorController = new CommandXboxController(1);
        public static String CAN_BUS_NAME = "6941CANivore1";

    }

    public static class SwerveConstants {

        public static final TunableNumber LongShotAngle = new TunableNumber("Long shot angle", 25);

        public static final Measure<Voltage> MAX_VOLTAGE = Volts.of(12.0);

        public static final double VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.0003;// 0.0003

        public static final int PIGEON_ID = 1;

        /**
         * The max speed of the swerve (should not larger than speedAt12Volts)
         */
        public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(4.5);
        /**
         * The max turning speed of the swerve
         */
        public static final Measure<Velocity<Angle>> maxAngularRate = RotationsPerSecond.of(1.5 * Math.PI);

        public static final double deadband = maxSpeed.magnitude() * 0.01;
        public static final double rotationalDeadband = maxAngularRate.magnitude() * 0.05;

        /**
         * Gearing between the drive motor output shaft and the wheel.
         */
        public static final double DRIVE_GEAR_RATIO = 6.7460317460317460317460317460317;
        /**
         * Theoretical free speed (m/s) at 12v applied output;
         */
        public static final Measure<Velocity<Distance>> speedAt12Volts = MetersPerSecond.of(4.5);
        public static final KinematicLimits DRIVETRAIN_UNCAPPED = new KinematicLimits(
                maxSpeed.magnitude(),
                25.0,
                5000);
        public static final SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(
                0.69522, 2.3623, 0.19367);
        public static final double statorCurrent = 110;
        public static final double supplyCurrent = 50;
        /**
         * Gearing between the steer motor output shaft and the azimuth gear.
         */
        private static final double STEER_GEAR_RATIO = 21.428571428571428571428571428571;
        /**
         * Radius of the wheel in meters.
         */
        private static final Measure<Distance> wheelRadius = Inches.of(1.8);//1.8752773878
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
        private static final double FRONT_LEFT_ENCODER_OFFSET = 0.2528613281;
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
        private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.2805136719;
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
        private static final double BACK_LEFT_ENCODER_OFFSET = 0.7752128906;
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
        private static final double BACK_RIGHT_ENCODER_OFFSET = 0.4254160156;
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

        public static final Translation2d[] modulePlacements = new Translation2d[]{
                new Translation2d(SwerveConstants.FrontLeft.LocationX,
                        SwerveConstants.FrontLeft.LocationY),
                new Translation2d(SwerveConstants.FrontRight.LocationX,
                        SwerveConstants.FrontRight.LocationY),
                new Translation2d(SwerveConstants.BackLeft.LocationX,
                        SwerveConstants.BackLeft.LocationY),
                new Translation2d(SwerveConstants.BackRight.LocationX,
                        SwerveConstants.BackRight.LocationY)
        };

        public static class steerGainsClass {
            public static final TunableNumber STEER_KP = new TunableNumber("STEER PID/kp", 120);
            public static final TunableNumber STEER_KI = new TunableNumber("STEER PID/ki", 0.2);
            public static final TunableNumber STEER_KD = new TunableNumber("STEER PID/kd", 0.005);
            public static final TunableNumber STEER_KA = new TunableNumber("STEER PID/ka", 0);
            public static final TunableNumber STEER_KV = new TunableNumber("STEER PID/kv", 0);
            public static final TunableNumber STEER_KS = new TunableNumber("STEER PID/ks", 0);
        }

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
            public static final TunableNumber HEADING_KI = new TunableNumber("HEADING PID/ki", 0.00);
            public static final TunableNumber HEADING_KD = new TunableNumber("HEADING PID/kd", 0.005);
            public static final TunableNumber MAX_ERROR_CORRECTION_ANGLE = new TunableNumber("HEADING/Max Error Correction Angle", 50.0);
            // TODO
        }
    }
}
