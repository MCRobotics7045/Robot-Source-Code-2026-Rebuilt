// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Region2d;

/** Add your docs here. */
public final class FieldConstants {

  public static final Translation2d BLUE_HUB_CENTER =
      new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
  public static final Translation2d RED_HUB_CENTER =
      new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));

  public static Translation2d getHubCenter(boolean isRed) {
    return isRed ? RED_HUB_CENTER : BLUE_HUB_CENTER;
  }

  public static final Region2d NEUTRAL_ZONE =
      new Region2d(
          new Translation2d[] {
            new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(-10)), // bottom-left
            new Translation2d(Units.inchesToMeters(445), Units.inchesToMeters(-10)), // bottom-right
            new Translation2d(Units.inchesToMeters(445), Units.inchesToMeters(330)), // top-right
            new Translation2d(Units.inchesToMeters(200), Units.inchesToMeters(330)) // top-left
          });
}
