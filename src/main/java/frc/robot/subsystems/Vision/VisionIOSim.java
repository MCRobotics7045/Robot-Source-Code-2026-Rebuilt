package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.CameraConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSim implements VisionIO {

  private final PhotonCamera camera;
  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim;
  private final Transform3d robotToCamera;
  private final Supplier<Pose2d> poseSupplier;

  public VisionIOSim(Supplier<Pose2d> poseSupplier, Transform3d robotToCamera) {
    this.poseSupplier = poseSupplier;
    this.robotToCamera = robotToCamera;

    camera = new PhotonCamera("SimCamera");
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(CameraConstants.aprilFeild);

    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(75));
    cameraProp.setCalibError(0.20, 0.05);
    cameraProp.setFPS(50);
    cameraProp.setAvgLatencyMs(25);
    cameraProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProp);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOinputs inputs) {
    visionSim.update(poseSupplier.get());

    inputs.CameraConnection = true;

    List<Integer> tagIds = new ArrayList<>();
    List<PoseObv> poseObservations = new ArrayList<>();

    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      if (result.hasTargets()) {
        PhotonTrackedTarget bestTarget = result.getBestTarget();
        inputs.latesTargetObv =
            new TargetObv(
                Rotation2d.fromDegrees(bestTarget.getYaw()),
                Rotation2d.fromDegrees(bestTarget.getPitch()));

        for (PhotonTrackedTarget target : result.targets) {
          tagIds.add(target.getFiducialId());
          Optional<Pose3d> tagPose = CameraConstants.aprilFeild.getTagPose(target.getFiducialId());

          if (tagPose.isPresent()) {
            Transform3d fieldToTarget =
                new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
            Transform3d cameraToTarget = target.getBestCameraToTarget();
            Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            Pose3d robotPose =
                new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

            poseObservations.add(
                new PoseObv(
                    result.getTimestampSeconds(),
                    robotPose,
                    target.getPoseAmbiguity(),
                    1,
                    cameraToTarget.getTranslation().getNorm(),
                    new short[] {(short) target.getFiducialId()}));
          }
        }
      }
    }

    inputs.poseObvs = poseObservations.toArray(new PoseObv[0]);
    inputs.tagID = tagIds.stream().mapToInt(Integer::intValue).toArray();
  }
}
