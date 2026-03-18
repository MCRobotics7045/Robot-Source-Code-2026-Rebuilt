// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private static final double HOOD_ENC_MIN = ShooterIOHoodMotor.ENCODER_MIN;
  private static final double HOOD_ENC_MAX = ShooterIOHoodMotor.ENCODER_MAX;

  private final ShooterIOinputsAutoLogged inputs = new ShooterIOinputsAutoLogged();

  private final ShooterIO ioMotor;
  private final ShooterIO ioHood;

  private double fPercent;

  public Shooter(ShooterIO ioMotor, ShooterIO ioHood) {
    this.ioMotor = ioMotor;
    this.ioHood = ioHood;
  }

  @Override
  public void periodic() {
    ioMotor.updateInputs(inputs);
    ioHood.updateInputs(inputs);
    Logger.processInputs("Shooter Inputs", inputs);

    fPercent = SmartDashboard.getNumber("Hood Angle", 0);
    SmartDashboard.putNumber("Hood Position (enc)", inputs.MotorHoodAngle);
    SmartDashboard.putBoolean("Hood At Setpoint", ioHood.isHoodAtSetpoint());
  }

  public double ProccesDistanceMotor(double Distance) {
    double targetVolts =
        MathUtil.clamp(ShooterConstants.kDistanceToVoltageMap.get(Distance), -12, 12);
    return targetVolts;
  }

  public double ProccesDistanceHoodAngle(double Distance) {
    double targetLength =
        MathUtil.clamp(ShooterConstants.kDistanceToAngleMap.get(Distance), 0, 1.5);
    return targetLength;
  }

  public void FireVoid(double Distance) {
    double MotorVoltage = ProccesDistanceMotor(Distance);
    double HoodRotations = ProccesDistanceHoodAngle(Distance);
    ioHood.setHoodPosition(HoodRotations);
    ioMotor.RunVoltage(MotorVoltage);
  }

  public Command FireCommand(Double Distance) {
    return this.startEnd(
        () -> FireVoid(Distance),
        () -> {
          ioHood.StopMotor();
          ioMotor.StopMotor();
        });
  }

  public Command FireBlankCommand(double Voltage) {
    return this.startEnd(() -> ioMotor.RunVoltage(Voltage), () -> ioMotor.StopMotor());
  }

  public Command hoodStop() {
    return this.runOnce(() -> ioHood.StopMotor());
  }

  public Command MotorStop() {
    return this.runOnce(() -> ioMotor.StopMotor());
  }

  /**
   * @param volts voltage to apply (positive = toward ENCODER_MAX)
   */
  public Command hoodRunVoltage(double volts) {
    return this.startEnd(() -> ioHood.setHoodVoltage(volts), () -> ioHood.StopMotor());
  }

  public Command hoodDistanceToPosition(double Distance) {
    double clampedTarget =
        MathUtil.clamp(
            ShooterConstants.kDistanceToAngleMap.get(Distance), HOOD_ENC_MIN, HOOD_ENC_MAX);
    return this.run(() -> ioHood.setHoodPosition(clampedTarget));
  }

  public Command hoodPidFromDashboard() {
    return this.run(
        () -> {
          double targetPosition = MathUtil.clamp(fPercent, HOOD_ENC_MIN, HOOD_ENC_MAX);
          ioHood.setHoodPosition(targetPosition);
        });
  }
}
