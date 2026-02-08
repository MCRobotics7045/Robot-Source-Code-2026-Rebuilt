// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Uptake;

import org.littletonrobotics.junction.AutoLog;

public interface UptakeIO {

  @AutoLog
  public static class UptakeIOInputs {
    public double MotorRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  default void updateInputs(UptakeIOInputs inputs) {}

  default void stopUptake() {}

  default void runUptake(double setSpeed) {}
}
