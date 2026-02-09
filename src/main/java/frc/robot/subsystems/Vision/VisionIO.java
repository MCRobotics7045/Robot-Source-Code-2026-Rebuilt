// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Transform3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOinputs {
    // Left Camera
    public int lBestTag = 0;
    public boolean lhasTargets = false;
    public double lposeAmbiguity = 0.0;
    public double lTargetYaw = 0.0;
    public double lTargetPitch = 0.0;
    public double lTargetArea = 0.0;
    public Transform3d ltargetDistance = null;
    public double ltimestamp = 0.0;

    // Right Camera
    public int rBestTag = 0;
    public boolean rhasTargets = false;
    public double rposeAmbiguity = 0.0;
    public double rTargetYaw = 0.0;
    public double rTargetPitch = 0.0;
    public double rTargetArea = 0.0;
    public Transform3d rtargetDistance = null;
    public double rtimestamp = 0.0;

    public Transform3d RrobotToCamera = null;
    public Transform3d LrobotToCamera = null;
  }

  default void updateInputs(VisionIOinputs inputs) {}
}
