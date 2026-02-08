// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIoinputs {
    public double MotorPos = 0.0;
    public double appliedVolts = 0.0;
    public double CurrentAmps = 0.0;
    public double DMotorRPM = 0.0;
    public double DappliedVolts = 0.0;
    public double DAmprege = 0.0;
  }

  default void updateInputs(IntakeIoinputs inputs) {}

  default void setIntakePostion(double ang) {}

  default void runIntakeD(double speed) {}

  default void stopIntakePos() {}

  default void stopIntakeD() {}
}
