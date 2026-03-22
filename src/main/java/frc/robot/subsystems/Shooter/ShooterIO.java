// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOinputs {
    public double MotorHoodAngle = 0.0;
    public double RequestedPostionPercent = 0.0;
    public double MotorRPM = 0.0;
    public double MotorVolts = 0.0;
    public double MotorAmp = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;
  }

  default void updateInputs(ShooterIOinputs inputs) {}

  default void ExtendAct() {}

  default void RetractAct() {}

  default void SetActuatorPercent(double percent) {}

  default void SetActuatorHeightMM(double MM) {}

  default void RunVoltage(double Voltage) {}

  default void VelocityPID() {}

  default void StopMotor() {}

  // Hood motor methods
  default void setHoodVoltage(double volts) {}

  default void setHoodPosition(double targetRotations) {}

  default boolean isHoodAtSetpoint() {
    return false;
  }

  default void SetRpm(double rpm) {}

  default void resetHoodEncoder() {}

  default boolean isAtSpeed() {
    return false;
  }
}
