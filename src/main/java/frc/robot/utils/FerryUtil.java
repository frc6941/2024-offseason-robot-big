package frc.robot.utils;

import frc.robot.utils.FieldLayout;
import frc.robot.utils.ShootingParameters.LaunchParameterTable;
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

	

	/**
	 * Get adaptive ferry shot parameters
	 * @param robot_pose Pose of the robot
	 * @param is_red_alliance
	 * @return Array of length 4 containing distance to target, hood angle, shooter rpm, and drivetrain angle
	 */
	public static double[] getFerryShotParameters(edu.wpi.first.math.geometry.Pose2d robot_pose_wpi, boolean is_red_alliance) {
		
		Translation2d robot_to_target =
				target.translateBy(robot_pose.getTranslation().inverse());
		Rotation2d target_drive_heading = robot_to_target
				.direction()
				.rotateBy(Rotation2d.fromDegrees(180.0))
				.rotateBy(Rotation2d.fromDegrees(-10.0));
				//TODO: WTF
		double arm_setpoint, velocity_setpoint, dist_to_target;
		dist_to_target = robot_to_target.norm();
		ShootingParameters high_parameter = LaunchParameterTable.getInstance().getParameters(dist_to_target);
		ShootingParameters low_parameter = LowFerryParameterTable.getInstance().getParameters(dist_to_target);
		if (inHighFerryZone(robot_pose, is_red_alliance) || midfield_target) {
			arm_setpoint = high_parameter.getAngle();
			velocity_setpoint = high_parameter.getVelocity();
		} else {
			arm_setpoint = low_parameter.getAngle();
			velocity_setpoint = low_parameter.getVelocity();
		} return new double[]{dist_to_target, arm_setpoint, velocity_setpoint, target_drive_heading.getDegrees()};
	}

	
}
