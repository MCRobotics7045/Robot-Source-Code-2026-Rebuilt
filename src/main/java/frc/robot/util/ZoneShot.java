package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;

public final class ZoneShot {

  private ZoneShot() {}

  /**
   * Get the target translation based on robot's current zone and alliance.
   *
   * @param robotPose Current robot pose
   * @param isRed True if on red alliance, false if blue
   * @return Target translation to aim at
   */
  public static Translation2d getTargetTranslation(Pose2d robotPose, boolean isRed) {
    if (FieldConstants.NEUTRAL_ZONE_LEFT.contains(robotPose)) {
      return isRed
          ? FieldConstants.NEUTRAL_LEFT_RED_TARGET
          : FieldConstants.NEUTRAL_LEFT_BLUE_TARGET;
    }
    if (FieldConstants.NEUTRAL_ZONE_RIGHT.contains(robotPose)) {
      return isRed
          ? FieldConstants.NEUTRAL_RIGHT_RED_TARGET
          : FieldConstants.NEUTRAL_RIGHT_BLUE_TARGET;
    }

    return FieldConstants.getHubCenter(isRed);
  }

  /**
   * Get the target angle for aiming at the zone-appropriate target.
   *
   * @param robotPose Current robot pose
   * @param isRed True if on red alliance, false if blue
   * @return Rotation2d pointing toward the target
   */
  public static Rotation2d getTargetAngle(Pose2d robotPose, boolean isRed) {
    var target = getTargetTranslation(robotPose, isRed);
    return target.minus(robotPose.getTranslation()).getAngle();
  }

  /**
   * Returns true if the robot is in either neutral zone (not in its own alliance zone).
   *
   * @param robotPose Current robot pose
   * @return True if in a neutral zone
   */
  public static boolean isInNeutralZone(Pose2d robotPose) {
    return FieldConstants.NEUTRAL_ZONE_LEFT.contains(robotPose)
        || FieldConstants.NEUTRAL_ZONE_RIGHT.contains(robotPose);
  }

  /**
   * Returns true if the robot is in the opposing alliance's zone.
   *
   * @param robotPose Current robot pose
   * @param isRed True if on red alliance, false if blue
   * @return True if in the opposing zone
   */
  public static boolean isInOpposingZone(Pose2d robotPose, boolean isRed) {
    return isRed
        ? FieldConstants.BLUE_ZONE.contains(robotPose)
        : FieldConstants.RED_ZONE.contains(robotPose);
  }
}
