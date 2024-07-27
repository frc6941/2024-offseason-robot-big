package frc.robot;

import org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.a.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class Constants {
	public static final boolean TUNING = true;
	public static final double LOOPER_DT = 1 / 100.0;

	public class RobotConstants {

		public static String CAN_BUS_NAME = "6941CANivore1";

		public static final CommandXboxController driverController = new CommandXboxController(0);

		public static final CommandXboxController operatorController = new CommandXboxController(1);

	}

	public class SwerveDrivetrian {

		public static final int PIGEON_ID = 1;

		public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
				.withCANbusName(RobotConstants.CAN_BUS_NAME)
				.withPigeon2Id(PIGEON_ID)
				.withPigeon2Configs(null); // optional

		/** The max speed of the swerve (should not larger than speedAt12Volts) */
		public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(6);
		/** The max turning speed of the swerve */
		public static final Measure<Velocity<Angle>> maxAngularRate = RotationsPerSecond.of(1.5 * Math.PI);

		public static final double deadband = maxSpeed.magnitude() * 0.1;
		public static final double rotationalDeadband = maxAngularRate.magnitude() * 0.1;

		public static final SlewRateLimiter xLimiter = new SlewRateLimiter(3, -3.25, 0);
		public static final SlewRateLimiter yLimiter = new SlewRateLimiter(3, -3.25, 0);

		/** Gearing between the drive motor output shaft and the wheel. */
		private static final double DRIVE_GEAR_RATIO = 6.7460317460317460317460317460317;
		/** Gearing between the steer motor output shaft and the azimuth gear. */
		private static final double STEER_GEAR_RATIO = 21.428571428571428571428571428571;

		/** Radius of the wheel in meters. */
		private static final Measure<Distance> wheelRadius = Meters.of(0.05);

		/** The stator current at which the wheels start to slip */
		private static final Measure<Current> slipCurrent = Amps.of(150.0);

		public static class steerGainsClass {
			public static final TunableNumber STEER_KP = new TunableNumber("STEER PID/kp", 120);
			public static final TunableNumber STEER_KI = new TunableNumber("STEER PID/ki", 0.2);
			public static final TunableNumber STEER_KD = new TunableNumber("STEER PID/kd", 0.005);
			public static final TunableNumber STEER_KA = new TunableNumber("STEER PID/ka", 0);
			public static final TunableNumber STEER_KV = new TunableNumber("STEER PID/kv", 0);
			public static final TunableNumber STEER_KS = new TunableNumber("STEER PID/ks", 0);
		}

		public static class driveGainsClass {
			public static final TunableNumber DRIVE_KP = new TunableNumber("DRIVE PID/kp", 0.03);
			public static final TunableNumber DRIVE_KI = new TunableNumber("DRIVE PID/ki", 0);
			public static final TunableNumber DRIVE_KD = new TunableNumber("DRIVE PID/kd", 0);
			public static final TunableNumber DRIVE_KA = new TunableNumber("DRIVE PID/ka", 0);
			public static final TunableNumber DRIVE_KV = new TunableNumber("DRIVE PID/kv", 0.12);
			public static final TunableNumber DRIVE_KS = new TunableNumber("DRIVE PID/ks", 0);
		}

		/** Swerve steering gains */
		private static final Slot0Configs steerGains = new Slot0Configs()
				.withKP(120)// 120
				.withKI(0.2)// 0.2
				.withKD(0.005)// 0.005
				.withKS(0)
				.withKV(0)
				.withKA(0);

		/** Swerve driving gains */
		private static final Slot0Configs driveGains = new Slot0Configs()
				.withKP(1)//0.3
				.withKI(0)
				.withKD(0)
				.withKS(0)
				.withKV(0.12)//2//0.00001
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

		/** Theoretical free speed (m/s) at 12v applied output; */
		public static final Measure<Velocity<Distance>> speedAt12Volts = MetersPerSecond.of(6.0);

		/** Simulation only */
		private static final double STEER_INERTIA = 0.00001;
		/** Simulation only */
		private static final double DRIVE_INERTIA = 0.001;
		/** Simulation only */
		private static final Measure<Voltage> steerFrictionVoltage = Volts.of(0.25);
		/** Simulation only */
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
				.withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.SyncCANcoder)
				.withCouplingGearRatio(COUPLE_RATIO)
				.withSteerMotorInverted(STEER_MOTOR_REVERSED);

		// Front Left
		private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 15;
		private static final int FRONT_LEFT_STEER_MOTOR_ID = 14;
		private static final int FRONT_LEFT_ENCODER_ID = 20;
		private static final double FRONT_LEFT_ENCODER_OFFSET = 0.053734;// 0.052955;//0.127686//0.5329550781
		private static final Measure<Distance> frontLeftXPos = Meters.of(0.5);
		private static final Measure<Distance> frontLeftYPos = Meters.of(0.5);

		// Front Right
		private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
		private static final int FRONT_RIGHT_STEER_MOTOR_ID = 7;
		private static final int FRONT_RIGHT_ENCODER_ID = 21;
		private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.127441;// 0.125685;//0.13623046875//0.117686//0.046875
		private static final Measure<Distance> frontRightXPos = Meters.of(0.5);
		private static final Measure<Distance> frontRightYPos = Meters.of(-0.5);

		// Back Left
		private static final int BACK_LEFT_DRIVE_MOTOR_ID = 5;
		private static final int BACK_LEFT_STEER_MOTOR_ID = 3;
		private static final int BACK_LEFT_ENCODER_ID = 9;
		private static final double BACK_LEFT_ENCODER_OFFSET = 0.775681;// 0.773925;//-0.223//0.401611//0.77392578125
		private static final Measure<Distance> backLeftXPos = Meters.of(-0.5);
		private static final Measure<Distance> backLeftYPos = Meters.of(0.5);

		// Back Right
		private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 10;
		private static final int BACK_RIGHT_STEER_MOTOR_ID = 6;
		private static final int BACK_RIGHT_ENCODER_ID = 12;
		private static final double BACK_RIGHT_ENCODER_OFFSET = 0.434814;// 0.422119;//-0.5684550781//-0.064453//0.432279296875
		private static final Measure<Distance> backRightXPos = Meters.of(-0.5);
		private static final Measure<Distance> backRightYPos = Meters.of(-0.5);

		public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
				FRONT_LEFT_STEER_MOTOR_ID,
				FRONT_LEFT_DRIVE_MOTOR_ID,
				FRONT_LEFT_ENCODER_ID,
				FRONT_LEFT_ENCODER_OFFSET,
				frontLeftXPos.magnitude(),
				frontLeftYPos.magnitude(),
				false);
		public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
				FRONT_RIGHT_STEER_MOTOR_ID,
				FRONT_RIGHT_DRIVE_MOTOR_ID,
				FRONT_RIGHT_ENCODER_ID,
				FRONT_RIGHT_ENCODER_OFFSET,
				frontRightXPos.magnitude(),
				frontRightYPos.magnitude(),
				true);
		public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
				BACK_LEFT_STEER_MOTOR_ID,
				BACK_LEFT_DRIVE_MOTOR_ID,
				BACK_LEFT_ENCODER_ID,
				BACK_LEFT_ENCODER_OFFSET,
				backLeftXPos.magnitude(),
				backLeftYPos.magnitude(),
				false);
		public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
				BACK_RIGHT_STEER_MOTOR_ID,
				BACK_RIGHT_DRIVE_MOTOR_ID,
				BACK_RIGHT_ENCODER_ID,
				BACK_RIGHT_ENCODER_OFFSET,
				backRightXPos.magnitude(),
				backRightYPos.magnitude(),
				true);

		public static SwerveModuleConstants[] modules = { FrontLeft, FrontRight, BackLeft, BackRight };

		public static final Translation2d[] modulePlacements = new Translation2d[] {
				new Translation2d(Constants.SwerveDrivetrian.FrontLeft.LocationX,
						Constants.SwerveDrivetrian.FrontLeft.LocationY),
				new Translation2d(Constants.SwerveDrivetrian.FrontRight.LocationX,
						Constants.SwerveDrivetrian.FrontRight.LocationY),
				new Translation2d(Constants.SwerveDrivetrian.BackLeft.LocationX,
						Constants.SwerveDrivetrian.BackLeft.LocationY),
				new Translation2d(Constants.SwerveDrivetrian.BackRight.LocationX,
						Constants.SwerveDrivetrian.BackRight.LocationY)
		};

		public static final KinematicLimits DRIVETRAIN_UNCAPPED = new KinematicLimits(
                4.5,
                50.0,
                2000.0);
        public static final KinematicLimits DRIVETRAIN_SMOOTHED = new KinematicLimits(
                4.5,
                30.0,
                200.0);
        public static final KinematicLimits DRIVETRAIN_LIMITED = new KinematicLimits(
                2.0,
                10.0,
                1200.0);
        public static final KinematicLimits DRIVETRAIN_ROBOT_ORIENTED = new KinematicLimits(
                2.0,
                5.0,
                1500.0
        );

		public static final SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(
                0.69522, 2.3623, 0.19367);

		/** Spin PID */
		public static final Slot0Configs headingGains = new Slot0Configs()
				.withKP(0.04)
				.withKI(0)
				.withKD(0);

	}

}
