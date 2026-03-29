// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionIO[] io;

  private final VisionIOinputsAutoLogged[] inputs;
  private final VisionConsumer consumer;
  private final PoseResetter poseResetter;
  private final Alert[] disconnectedAlerts;

  private final List<Pose3d> tagPoses = new ArrayList<>();
  private final List<Pose3d> robotPoses = new ArrayList<>();
  private final List<Pose3d> acceptedPoses = new ArrayList<>();
  private final List<Pose3d> rejectedPoses = new ArrayList<>();
  private boolean initialPoseSet = false;

  // Pre-computed logger keys — avoids string allocation every loop
  private final String[] cameraLogKeys;
  private final String[] tagPosesKeys;
  private final String[] robotPosesKeys;
  private final String[] acceptedPosesKeys;
  private final String[] rejectedPosesKeys;

  public Vision(VisionConsumer consumer, PoseResetter poseResetter, VisionIO... io) {
    this.io = io;
    this.consumer = consumer;
    this.poseResetter = poseResetter;

    this.inputs = new VisionIOinputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOinputsAutoLogged();
    }

    this.disconnectedAlerts = new Alert[io.length];
    cameraLogKeys = new String[io.length];
    tagPosesKeys = new String[io.length];
    robotPosesKeys = new String[io.length];
    acceptedPosesKeys = new String[io.length];
    rejectedPosesKeys = new String[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] = new Alert("Vision Camera" + i + "Disconnected.", AlertType.kWarning);
      cameraLogKeys[i] = "Vision/Camera" + i;
      tagPosesKeys[i] = "Vision/Camera" + i + "/TagPoses";
      robotPosesKeys[i] = "Vision/Camera" + i + "/RobotPoses";
      acceptedPosesKeys[i] = "Vision/Camera" + i + "/AcceptedPoses";
      rejectedPosesKeys[i] = "Vision/Camera" + i + "/RejectedPoses";
    }

    // Log camera poses relative to robot for visualization in AdvantageScope
    Logger.recordOutput(
        "Vision/CameraL/PoseOnRobot",
        new Pose3d().transformBy(CameraConstants.CAMERA_L_TRANSFORM_TO_ROBOT));
    Logger.recordOutput(
        "Vision/CameraR/PoseOnRobot",
        new Pose3d().transformBy(CameraConstants.CAMERA_R_TRANSFORM_TO_ROBOT));
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs(cameraLogKeys[i], inputs[i]);
    }

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].CameraConnection);
      tagPoses.clear();
      robotPoses.clear();
      acceptedPoses.clear();
      rejectedPoses.clear();
      for (int tagId : inputs[cameraIndex].tagID) {
        Optional<Pose3d> tagPose = CameraConstants.aprilFeild.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      for (VisionIO.PoseObv observation : inputs[cameraIndex].poseObvs) {
        boolean reject =
            observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.Ambiguity() > 0.3)
                || observation.pose().getX() < 0.0
                || observation.pose().getY() < 0.0
                || observation.pose().getX() > CameraConstants.aprilFeild.getFieldLength()
                || observation.pose().getY() > CameraConstants.aprilFeild.getFieldWidth();

        robotPoses.add(observation.pose());

        if (reject) {
          rejectedPoses.add(observation.pose());
          continue;
        }

        acceptedPoses.add(observation.pose());

        // Snap pose on first confident multi-tag fix so the estimator doesn't
        // have to converge from Pose2d.kZero.
        if (!initialPoseSet && observation.tagCount() >= 2) {
          poseResetter.resetPose(observation.pose().toPose2d());
          initialPoseSet = true;
          Logger.recordOutput("Vision/InitialPoseSet", true);
        }

        Matrix<N3, N1> curStdDevs;
        double distanceScale =
            1 + (observation.avgTagDistance() * observation.avgTagDistance() / 30);

        if (observation.tagCount() == 1 && observation.avgTagDistance() > 3.0) {
          // Single tag too far away — reject entirely
          curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else if (observation.tagCount() > 1 && observation.avgTagDistance() <= 2.5) {
          // High-confidence multi-tag at close range: trust x/y AND rotation for drift correction
          curStdDevs = CameraConstants.kMultiTagStdDevs.times(distanceScale);
        } else {
          // Single tag or far multi-tag: trust x/y position only, ignore rotation
          Matrix<N3, N1> base =
              observation.tagCount() > 1
                  ? CameraConstants.kMultiTagStdDevs
                  : CameraConstants.kSingleTagStdDevs;
          curStdDevs =
              VecBuilder.fill(
                  base.get(0, 0) * distanceScale, base.get(1, 0) * distanceScale, Double.MAX_VALUE);
        }

        consumer.accept(observation.pose().toPose2d(), observation.time(), curStdDevs);
      }

      Logger.recordOutput(tagPosesKeys[cameraIndex], tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(robotPosesKeys[cameraIndex], robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(acceptedPosesKeys[cameraIndex], acceptedPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(rejectedPosesKeys[cameraIndex], rejectedPoses.toArray(new Pose3d[0]));
    }
  }

  /** Returns true if any camera has lost connection to PhotonVision. */
  public boolean isAnyCameraDisconnected() {
    for (int i = 0; i < inputs.length; i++) {
      if (!inputs[i].CameraConnection) return true;
    }
    return false;
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeterPose2d,
        double getTimestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  @FunctionalInterface
  public interface PoseResetter {
    void resetPose(Pose2d pose);
  }
}
