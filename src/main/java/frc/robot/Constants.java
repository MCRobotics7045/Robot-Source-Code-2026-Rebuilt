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

  public static final int ShooterBeamBreakDIOChannel = 1;

  public static final double DRIVETRAIN_TURN_SPEED_MODIFIER = 0.5;

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

    public static final Transform3d CAMERA_R_TRANSFORM_TO_ROBOT = // RIGHT SIDE LOOKING AT FRONT
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(7.527502), // FWD: positive = front of robot  X
                Units.inchesToMeters(8.822286), // RIGHT side (negative Y)  Y
                Units.inchesToMeters(18.664261)), // HEIGHT: on top   Z
            new Rotation3d(
                Units.degreesToRadians(0), // ROLL
                Units.degreesToRadians(-25), // PITCH: angled up (negative = nose up)
                Units.degreesToRadians(
                    10))); // YAW: 10 degrees outward (right) from straight forward

    public static final Transform3d CAMERA_L_TRANSFORM_TO_ROBOT =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(7.527502), // FWD: positive = front of robot
                Units.inchesToMeters(-8.822286), // LEFT side (positive Y)
                Units.inchesToMeters(18.664261)), // HEIGHT: on top
            new Rotation3d(
                Units.degreesToRadians(0), // ROLL
                Units.degreesToRadians(-25), // PITCH: angled up (negative = nose up)
                Units.degreesToRadians(
                    -10))); // YAW: 10 degrees outward (left) from straight forward

    public static final Matrix<N3, N1> kSingleTagStdDevs =
        VecBuilder.fill(1.0, 1.0, Double.MAX_VALUE);

    // Rotation only trusted when multi-tag + close range (see Vision.java)
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.3, 0.3, 0.2);
  }

  public static final class ShooterConstants {
    public static final InterpolatingDoubleTreeMap kDistanceToRPMmap =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap kDistanceToAngleMap =
        new InterpolatingDoubleTreeMap();

    // Fallback values used when any PhotonVision camera is disconnected during auto-fire
    public static final double NO_VISION_FALLBACK_RPM = -3200.0;
    public static final double NO_VISION_FALLBACK_HOOD = 0.2;

    // Fixed shot values used when the robot is in a neutral zone
    public static final double NEUTRAL_ZONE_RPM = -4000.0;
    public static final double NEUTRAL_ZONE_HOOD = 0.7;

    // Fixed shot values used when the robot is in the opposing alliance zone
    public static final double OPPOSING_ZONE_RPM = -4500.0;
    public static final double OPPOSING_ZONE_HOOD = 1.2;

    static {
      kDistanceToRPMmap.put(1.02, 3000.0);
      kDistanceToAngleMap.put(1.02, 0.1);
      kDistanceToRPMmap.put(2.03, 3300.0);
      kDistanceToAngleMap.put(2.03, 0.25);
      kDistanceToRPMmap.put(3.05, 3300.0);
      kDistanceToAngleMap.put(3.05, 0.55);
      kDistanceToRPMmap.put(4.25, 3690.0);
      kDistanceToAngleMap.put(4.25, 0.6);
    }
  }

  public static final class AllianceShiftConstants {
    // Match time remaining (seconds) at each shift boundary during teleop
    public static final double TRANSITION_TIME = 130.0;
    public static final double SHIFT_1_TIME = 105.0;
    public static final double SHIFT_2_TIME = 80.0;
    public static final double SHIFT_3_TIME = 55.0;
    public static final double END_GAME_TIME = 30.0;
  }

  public static final class MotorConstants {
    // Intake POS
    public static final double IntakeStowed = 0; // stowed postion (should be 0)
    public static final double IntakeCollect = 6.4; // Intake Out Position
    public static final double MaxShutter = 3; // Max Shutter angle(proably could be less)

    // Intake Drive
    public static final double IntakeMaxSpeed = 0.7; // Speed of roller wheels
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
