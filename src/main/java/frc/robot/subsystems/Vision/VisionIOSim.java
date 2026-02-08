// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {

  private VisionSystemSim visionSim;
  private TargetModel targetModel;
  private AprilTagFieldLayout tagLayout;
  private static AprilTagFieldLayout kTagLayout = null;
  private SimCameraProperties cameraProp;

  // FAKE CAM
  private PhotonCamera lCamera;
  private PhotonCamera rCamera;

  PhotonCameraSim lCameraSim;
  PhotonCameraSim rCameraSim;

  Translation3d RcameraPos =
      new Translation3d(
          Units.inchesToMeters(-3.010),
          Units.inchesToMeters(-13.027),
          Units.inchesToMeters(19.431));

  Rotation3d RcameraRot =
      new Rotation3d(0.0, Units.degreesToRadians(-25), Units.degreesToRadians(10));

  Transform3d RrobotToCamera = new Transform3d(RcameraPos, RcameraRot);

  private Supplier<Pose2d> poseSupplier;

  public VisionIOSim(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
    lCamera = new PhotonCamera("LeftCam");
    rCamera = new PhotonCamera("RightCam");
    visionSim = new VisionSystemSim("main");
    targetModel = (TargetModel.kAprilTag36h11);
    cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(75));
    cameraProp.setCalibError(0.20, 0.05);
    cameraProp.setFPS(50);
    cameraProp.setAvgLatencyMs(25);
    cameraProp.setLatencyStdDevMs(5);
    lCameraSim = new PhotonCameraSim(lCamera, cameraProp);
    rCameraSim = new PhotonCameraSim(rCamera, cameraProp);
    visionSim.addAprilTags(getTagLayout());
    visionSim.addCamera(rCameraSim, RrobotToCamera);
  }

  public void updateInputs(VisionIOinputs inputs) {
    visionSim.update(poseSupplier.get());
    inputs.RrobotToCamera = RrobotToCamera;
  }

  private static AprilTagFieldLayout getTagLayout() {
    if (kTagLayout == null) {
      kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    }
    return kTagLayout;
  }
}
