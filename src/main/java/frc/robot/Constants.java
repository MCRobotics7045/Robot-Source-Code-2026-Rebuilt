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
                Units.inchesToMeters(7.673), // FWD: positive = front of robot
                Units.inchesToMeters(-8.980265), // RIGHT side (negative Y)
                Units.inchesToMeters(17.289050)), // HEIGHT: on top
            new Rotation3d(
                Units.degreesToRadians(-10), // ROLL: banked outward to the right
                Units.degreesToRadians(-26.805957), // PITCH: angled up (negative = nose up)
                Units.degreesToRadians(0))); // YAW: facing forward

    public static final Transform3d CAMERA_L_TRANSFORM_TO_ROBOT =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(7.673), // FWD: positive = front of robot
                Units.inchesToMeters(8.980265), // LEFT side (positive Y)
                Units.inchesToMeters(17.289050)), // HEIGHT: on top
            new Rotation3d(
                Units.degreesToRadians(10), // ROLL: banked outward to the left
                Units.degreesToRadians(-26.805957), // PITCH: angled up (negative = nose up)
                Units.degreesToRadians(0))); // YAW: facing forward

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 2);

    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, 0.5);
  }

  public static final class ShooterConstants {
    public static final InterpolatingDoubleTreeMap kDistanceToVoltageMap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToAngleMap =
        new InterpolatingDoubleTreeMap();

    static {
      kDistanceToVoltageMap.put(2.0, -4.0);
      kDistanceToAngleMap.put(2.0, 0.5);
      kDistanceToVoltageMap.put(1.0, 6.0);
      kDistanceToAngleMap.put(1.0, 1.0);
    }
  }

  public static final class MotorConstants {
    // Intake POS
    public static final double IntakeStowed = 0; // stowed postion (should be 0)
    public static final double IntakeCollect = 122; // Intake Out Position
    public static final double MaxShutter = 3; // Max Shutter angle(proably could be less)

    // Intake Drive
    public static final double IntakeMaxSpeed = 0.5; // Speed of roller wheels
    public static final double ShutterSpeed =
        0.5; // How fast to mvoe between each angle for shutter

    // ID
    public static final int ShooterMotorID = 20;
    public static final int HoodMotorID = 35;
    public static final int IntakePosMotorID = 37;
    public static final int IntakeDrivMotorID = 38;
    public static final int IndexerBeltMotorID = 33;
    public static final int IndexerStarMotorID = 31;
  }
}
