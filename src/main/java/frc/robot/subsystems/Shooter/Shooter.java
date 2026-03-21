// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private static final double HOOD_ENC_MIN = ShooterIOHoodMotor.ENCODER_MIN;
  private static final double HOOD_ENC_MAX = ShooterIOHoodMotor.ENCODER_MAX;

  private final ShooterIOinputsAutoLogged inputs = new ShooterIOinputsAutoLogged();

  private final ShooterIO ioMotor;
  private final ShooterIO ioHood;

  private double voltz;
  private double fPercent;

  public Shooter(ShooterIO ioMotor, ShooterIO ioHood) {
    this.ioMotor = ioMotor;
    this.ioHood = ioHood;
    SmartDashboard.putNumber("MotorVoltage", 0);
    SmartDashboard.putNumber("Hood Angle", 0);
  }

  @Override
  public void periodic() {
    ioMotor.updateInputs(inputs);
    ioHood.updateInputs(inputs);
    Logger.processInputs("Shooter Inputs", inputs);
    voltz = SmartDashboard.getNumber("MotorVoltage", 0);
    fPercent = SmartDashboard.getNumber("Hood Angle", 0);
    SmartDashboard.putNumber("Hood Position (enc)", inputs.MotorHoodAngle);
    SmartDashboard.putBoolean("Hood At Setpoint", ioHood.isHoodAtSetpoint());
  }

  public double ProccesDistanceMotor(DoubleSupplier Distance) {
    double targetVolts =
        MathUtil.clamp(ShooterConstants.kDistanceToVoltageMap.get(Distance.getAsDouble()), -12, 12);
    return targetVolts;
  }

  public double ProccesDistanceHoodAngle(DoubleSupplier Distance) {
    double targetLength =
        MathUtil.clamp(
            ShooterConstants.kDistanceToAngleMap.get(Distance.getAsDouble()), 0, HOOD_ENC_MAX);
    return targetLength;
  }

  public void FireVoid(DoubleSupplier Distance) {
    double MotorVoltage = ProccesDistanceMotor(Distance);
    double HoodRotations = ProccesDistanceHoodAngle(Distance);
    ioHood.setHoodPosition(HoodRotations);
    ioMotor.RunVoltage(MotorVoltage);
  }

  public Command FireCommand(DoubleSupplier Distance) {
    return this.startEnd(
        () -> FireVoid(Distance),
        () -> {
          ioHood.StopMotor();
          ioMotor.StopMotor();
        });
  }

  public Command FireBlankCommand() {
    return this.run(
            () -> {
              ioMotor.RunVoltage(voltz);
              ioHood.setHoodPosition(fPercent);
            })
        .finallyDo(
            () -> {
              ioMotor.StopMotor();
              ioHood.StopMotor();
            });
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

  public Command hoodDistanceToPosition(DoubleSupplier Distance) {
    return this.run(
            () -> {
              double clampedTarget =
                  MathUtil.clamp(
                      ShooterConstants.kDistanceToAngleMap.get(Distance.getAsDouble()),
                      HOOD_ENC_MIN,
                      HOOD_ENC_MAX);
              ioHood.setHoodPosition(clampedTarget);
            })
        .finallyDo(() -> ioHood.StopMotor());
  }

  public Command hoodPidFromDashboard() {
    return this.run(
        () -> {
          double targetPosition = MathUtil.clamp(fPercent, HOOD_ENC_MIN, HOOD_ENC_MAX);
          ioHood.setHoodPosition(targetPosition);
        });
  }
}
