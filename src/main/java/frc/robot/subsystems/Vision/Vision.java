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
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionIO[] io;

  private final VisionIOinputsAutoLogged[] inputs;
  private final VisionConsumer consumer;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.io = io;
    this.consumer = consumer;

    this.inputs = new VisionIOinputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOinputsAutoLogged();
    }

    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] = new Alert("Vision Camera" + i + "Disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].CameraConnection);
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> acceptedPoses = new LinkedList<>();
      List<Pose3d> rejectedPoses = new LinkedList<>();

      for (int tagId : inputs[cameraIndex].tagID) {
        Optional<Pose3d> tagPose = CameraConstants.aprilFeild.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      for (VisionIO.PoseObv observation : inputs[cameraIndex].poseObvs) {
        boolean reject =
            observation.tagCount() == 0
                || (observation.tagCount() == 1 && observation.Ambiguity() > 0.1)
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

        Matrix<N3, N1> stdDevs = CameraConstants.kSingleTagStdDevs;
        if (observation.tagCount() > 1) {
          stdDevs = CameraConstants.kMultiTagStdDevs;
        }

        if (observation.tagCount() == 1 && observation.avgTagDistance() > 3.0) {
          stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          stdDevs =
              stdDevs.times(1 + (observation.avgTagDistance() * observation.avgTagDistance() / 30));
        }

        consumer.accept(observation.pose().toPose2d(), observation.time(), stdDevs);
      }

      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/AcceptedPoses", acceptedPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/Camera" + cameraIndex + "/RejectedPoses", rejectedPoses.toArray(new Pose3d[0]));
    }
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeterPose2d,
        double getTimestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
