package frc.robot.utils;

import frc.robot.utils.FieldLayout;
import frc.robot.utils.ShootingParameters.HighFerryParameterTable;
import frc.robot.utils.ShootingParameters.LowFerryParameterTable;
import frc.robot.utils.ShootingParameters.ShootingParameters;
import frc.robot.utils.ShootingParameters.SpeakerParameterTable;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.Util;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.InterpolatingDouble;


public class FerryUtil {

	private static final double kOppoWingToAllianceWall =
			FieldLayout.distanceFromAllianceWall(FieldLayout.kWingX, true);
	public static final Translation2d kCornerTarget = new Translation2d(1.0, FieldLayout.kFieldWidth - 1.5);
	public static final Translation2d kMidTarget =
			new Translation2d((FieldLayout.kFieldLength / 2.0) - 1.0, FieldLayout.kFieldWidth - 0.2);

	/**
	 * Get adaptive ferry shot parameters
	 * @param robot_pose Pose of the robot
	 * @param is_red_alliance
	 * @return Array of length 4 containing distance to target, hood angle, shooter rpm, and drivetrain angle
	 */
	public static double[] getFerryShotParameters(edu.wpi.first.math.geometry.Pose2d robot_pose_wpi, boolean is_red_alliance) {
		Pose2d robot_pose = new Pose2d(robot_pose_wpi.getX(), robot_pose_wpi.getY(), Rotation2d.fromDegrees(robot_pose_wpi.getRotation().getDegrees()));
		boolean midfield_target = useMidfieldTarget(robot_pose.getTranslation().x(), is_red_alliance);
		Translation2d target;
		if (midfield_target) {
			target = FieldLayout.handleAllianceFlip(kMidTarget, is_red_alliance);
		} else {
			target = FieldLayout.handleAllianceFlip(kCornerTarget, is_red_alliance);
		}
		Translation2d robot_to_target =
				target.translateBy(robot_pose.getTranslation().inverse());
		Rotation2d target_drive_heading = robot_to_target
				.direction()
				.rotateBy(Rotation2d.fromDegrees(180.0))
				.rotateBy(Rotation2d.fromDegrees(-10.0));
				//TODO: WTF
		double arm_setpoint, velocity_setpoint, dist_to_target;
		dist_to_target = robot_to_target.norm();
		ShootingParameters high_parameter = HighFerryParameterTable.getInstance().getParameters(dist_to_target);
		ShootingParameters low_parameter = LowFerryParameterTable.getInstance().getParameters(dist_to_target);
		if (inHighFerryZone(robot_pose, is_red_alliance) || midfield_target) {
			arm_setpoint = high_parameter.getAngle();
			velocity_setpoint = high_parameter.getVelocity();
		} else {
			arm_setpoint = low_parameter.getAngle();
			velocity_setpoint = low_parameter.getVelocity();
		} return new double[]{dist_to_target, arm_setpoint, velocity_setpoint, target_drive_heading.getDegrees()};
	}

	private static boolean inHighFerryZone(Pose2d robot_pose, boolean is_red_alliance) {
		double x = robot_pose.getTranslation().x();
		double y = robot_pose.getTranslation().y();
		Translation2d cor_0 =
				FieldLayout.handleAllianceFlip(new Translation2d(FieldLayout.kWingX, 0.0), is_red_alliance);
		Translation2d cor_1 = FieldLayout.kCenterNote2;
		boolean in_x = Util.inRange(x, Math.min(cor_0.x(), cor_1.x()), Math.max(cor_0.x(), cor_1.x()));
		boolean in_y = Util.inRange(y, Math.min(cor_0.y(), cor_1.y()), Math.max(cor_0.y(), cor_1.y()));
		return in_x && in_y;
	}

	private static boolean useMidfieldTarget(double x_coord, boolean is_red_alliance) {
		return FieldLayout.distanceFromAllianceWall(x_coord, is_red_alliance) - kOppoWingToAllianceWall > 1.0;
	}
}
