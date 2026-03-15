// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  // Where Robot Most Likely is based
  record PoseObv(
      double time,
      Pose3d pose,
      double Ambiguity,
      int tagCount,
      double avgTagDistance,
      short[] tagID) {}

  record TargetObv(Rotation2d y, Rotation2d x) {}

  @AutoLog
  public static class VisionIOinputs {
    public boolean CameraConnection = false;

    public TargetObv latesTargetObv = new TargetObv(new Rotation2d(), new Rotation2d());

    public PoseObv[] poseObvs = new PoseObv[0];

    public int[] tagID = new int[0];
  }

  default void updateInputs(VisionIOinputs inputs) {}
}
