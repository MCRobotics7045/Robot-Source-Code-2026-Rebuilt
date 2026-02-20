// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static class CameraConstants {
    public static final AprilTagFieldLayout aprilFeild =
        AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField();

    public static final Transform3d CAMERA_R_TRANSFORM_TO_ROBOT =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-3.010),
                Units.inchesToMeters(-13.027),
                Units.inchesToMeters(19.431)),
            new Rotation3d(10.0, Units.degreesToRadians(-25), Units.degreesToRadians(10)));

    public static final Transform3d CAMERA_L_TRANSFORM_TO_ROBOT =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-3.010),
                Units.inchesToMeters(13.027),
                Units.inchesToMeters(19.431)),
            new Rotation3d(0.0, Units.degreesToRadians(-25), Units.degreesToRadians(-10)));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(5, 5, 8);

    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class ShooterConstants {
    public static final InterpolatingDoubleTreeMap kDistanceToVoltageMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToAngleMap =
        new InterpolatingDoubleTreeMap();

    static {
      kDistanceToVoltageMap.put(2.0, 4.0);
      kDistanceToAngleMap.put(2.0, 40.0);
      kDistanceToVoltageMap.put(1.0, 6.0);
      kDistanceToAngleMap.put(1.0, 50.0);
    }
  }
}
