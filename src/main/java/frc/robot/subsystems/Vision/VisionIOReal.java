// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.CameraConstants;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {

  private PhotonCamera camera;

  private final Transform3d robotToCamera;

  public VisionIOReal(String camName, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(camName);

    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOinputs inputs) {
    inputs.CameraConnection = (camera.isConnected());
    Set<Short> tagIds = new HashSet<>();
    List<PoseObv> poseObservations = new LinkedList<>();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        inputs.latesTargetObv =
            new TargetObv(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      }

      if (result.multitagResult.isPresent()) {
        MultiTargetPNPResult multiTargetPNPResult = result.multitagResult.get();
        Transform3d feildToCamera = multiTargetPNPResult.estimatedPose.best;
        Transform3d feildToRobot = feildToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(feildToRobot.getTranslation(), feildToRobot.getRotation());
        double TrargetDist = 0.0;

        for (PhotonTrackedTarget target : result.targets) {
          TrargetDist += target.bestCameraToTarget.getTranslation().getNorm();
        }
        tagIds.addAll(multiTargetPNPResult.fiducialIDsUsed);

        poseObservations.add(
            new PoseObv(
                result.getTimestampSeconds(),
                robotPose,
                multiTargetPNPResult.estimatedPose.ambiguity,
                multiTargetPNPResult.fiducialIDsUsed.size(),
                TrargetDist / result.targets.size(),
                multiTargetPNPResult.fiducialIDsUsed));
      } else if (!result.targets.isEmpty()) {
        PhotonTrackedTarget target = result.targets.get(0);

        Optional<Pose3d> tagPose = CameraConstants.aprilFeild.getTagPose(target.getFiducialId());

        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d CameraToTarget = target.bestCameraToTarget;
          Transform3d feildToCam = fieldToTarget.plus(CameraToTarget.inverse());
          Transform3d feildToRobot = feildToCam.plus(robotToCamera.inverse());

          Pose3d robotPose = new Pose3d(feildToRobot.getTranslation(), feildToRobot.getRotation());

          tagIds.add((short) target.fiducialId);

          poseObservations.add(
              new PoseObv(
                  result.getTimestampSeconds(),
                  robotPose,
                  target.poseAmbiguity,
                  1,
                  CameraToTarget.getTranslation().getNorm(),
                  List.of((short) target.getFiducialId())));
        }
      }
    }

    inputs.poseObvs = poseObservations.toArray(new PoseObv[0]);
    inputs.tagID = new int[tagIds.size()];

    int i = 0;
    for (int id : tagIds) {
      inputs.tagID[i++] = id;
    }
  }
}
