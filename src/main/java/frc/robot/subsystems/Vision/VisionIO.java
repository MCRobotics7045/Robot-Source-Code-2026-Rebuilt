// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOinputs {
    public int BestTag = 0;
    public boolean hasTargets = false;
    public double poseAmbiguity = 0.0;
    public double TargetYaw = 0.0;
    public double TargetPitch = 0.0;
    public double TargetArea = 0.0;
    public Transform3d RrobotToCamera = null;
  }

  default void updateInputs(VisionIOinputs inputs) {}
}
