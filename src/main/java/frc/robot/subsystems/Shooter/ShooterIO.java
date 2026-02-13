// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOinputs {
    public double LienarActuatorPos = 0.0;
    public double HoodAngle = 0.0;
    public double ReqActuatorPos = 0.0;
    public double lMotorRPM = 0.0;
    public double rMotorRPM = 0.0;
    public double lMotorVolts = 0.0;
    public double rMotorVolts = 0.0;
    public double lMotorAmp = 0.0;
    public double rMotorAmp = 0.0;
  }

  default void updateInputs(ShooterIOinputs inputs) {}

  default void ExtendAct() {}

  default void RetractAct() {}

  default void SetActuatorHeight(double Height) {}

  default void RunVoltage(double Voltage) {}

  default void VelocityPID() {}

  default void StopMotor() {}
}
