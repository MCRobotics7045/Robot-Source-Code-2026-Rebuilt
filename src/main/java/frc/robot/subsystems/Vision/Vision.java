// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  private final VisionIO io;

  private final VisionIOinputsAutoLogged inputs = new VisionIOinputsAutoLogged();
  private Supplier<Pose2d> robotPose;

  public Vision(VisionIO io, Supplier<Pose2d> robotPose) {
    this.io = io;
    this.robotPose = robotPose;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
    Pose2d currentRobotPose = robotPose.get();
    Pose3d cameraPose = new Pose3d(currentRobotPose).transformBy(inputs.RrobotToCamera);
    Logger.recordOutput("Vision/CameraPose", cameraPose);
  }
}
