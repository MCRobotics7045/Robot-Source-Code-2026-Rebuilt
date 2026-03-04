// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class FieldConstants {

  public static final Translation2d BLUE_HUB_CENTER =
      new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
  public static final Translation2d RED_HUB_CENTER =
      new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));


public static Translation2d getHubCenter(boolean isRed){
    return isRed ? RED_HUB_CENTER : BLUE_HUB_CENTER;
}

}

