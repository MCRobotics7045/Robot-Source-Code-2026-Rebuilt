// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private static final double HOOD_ENC_MIN = ShooterIOHoodMotor.ENCODER_MIN;
  private static final double HOOD_ENC_MAX = ShooterIOHoodMotor.ENCODER_MAX;

  private final ShooterIOinputsAutoLogged inputs = new ShooterIOinputsAutoLogged();

  private final ShooterIO ioMotor;
  private final ShooterIO ioHood;

  private double manualHoodPos = 0.0;

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
    // voltz = SmartDashboard.getNumber("MotorVoltage", 0);
    // fPercent = SmartDashboard.getNumber("Hood Angle", 0);
    SmartDashboard.putNumber("Hood Position (enc)", inputs.MotorHoodAngle);
    SmartDashboard.putBoolean("Hood At Setpoint", ioHood.isHoodAtSetpoint());
  }

  public double ProccesRPMmotor(DoubleSupplier Distance) {
    double targetRPM =
        MathUtil.clamp(ShooterConstants.kDistanceToRPMmap.get(Distance.getAsDouble()), -6065, 6065);
    return targetRPM;
  }

  public double ProccesDistanceHoodAngle(DoubleSupplier Distance) { // FIX
    double targetAngle =
        MathUtil.clamp(
            ShooterConstants.kDistanceToAngleMap.get(Distance.getAsDouble()), 0, HOOD_ENC_MAX);
    return targetAngle;
  }

  public Command hoodStop() {

    return Commands.sequence(
        this.run(
                () -> {
                  ioHood.setHoodPosition(0);
                  ioMotor.StopMotor();
                })
            .withTimeout(2),
        this.runOnce(() -> ioHood.StopMotor()));
  }

  public Command MotorStop() {
    return this.runOnce(() -> ioMotor.StopMotor());
  }

  public Command hoodDistanceToPosition(DoubleSupplier Distance) {
    return this.run(() -> ioHood.setHoodPosition(ProccesDistanceHoodAngle(Distance)));
  }

  public Command motorDistanceToPostion(DoubleSupplier Distance) {
    return this.run(() -> ioMotor.SetRpm(ProccesRPMmotor(Distance)));
  }

  public Command shooterDistanceToPosition(DoubleSupplier Distance) {
    return this.run(
        () -> {
          ioHood.setHoodPosition(ProccesDistanceHoodAngle(Distance));
          ioMotor.SetRpm(-ProccesRPMmotor(Distance));
        });
  }

  public Command shooterDistanceOrFallback(
      DoubleSupplier distance, BooleanSupplier visionOk, double fallbackHood, double fallbackRPM) {

    return this.run(
        () -> {
          if (visionOk.getAsBoolean()) {
            ioHood.setHoodPosition(ProccesDistanceHoodAngle(distance));
            ioMotor.SetRpm(-ProccesRPMmotor(distance));
          } else {
            ioHood.setHoodPosition(fallbackHood);
            ioMotor.SetRpm(fallbackRPM);
          }
        });
  }

  public boolean isShooterAtSpeed() {
    return ioMotor.isAtSpeed();
  }

  public Command StowHood() {
    return this.runOnce(
        () -> {
          ioHood.setHoodPosition(0);
          ioMotor.StopMotor();
        });
  }

  public Command ResetEncoder() {
    return this.runOnce(() -> ioHood.resetHoodEncoder());
  }

  public Command ShooterDirectRPM(double RPM) {
    double clampedRPM = MathUtil.clamp(RPM, -6065, 6065);
    return this.run(() -> ioMotor.SetRpm(clampedRPM));
  }

  public Command HoodDirectPostion(double Postion) {
    double clampedPostion = MathUtil.clamp(Postion, HOOD_ENC_MIN, HOOD_ENC_MAX);
    return this.run(() -> ioHood.setHoodPosition(clampedPostion));
  }

  public Command PresetShot(DoubleSupplier position, DoubleSupplier rpm) {
    return this.run(
        () -> {
          double clampedRPM = MathUtil.clamp(rpm.getAsDouble(), -6065, 6065);
          double clampedPos = MathUtil.clamp(position.getAsDouble(), HOOD_ENC_MIN, HOOD_ENC_MAX);
          ioMotor.SetRpm(clampedRPM);
          ioHood.setHoodPosition(clampedPos);
        });
  }

  public Command ManualHoodAdjust(double deltaPerLoop) {
    return new FunctionalCommand(
        () -> manualHoodPos = inputs.MotorHoodAngle,
        () -> {
          manualHoodPos = MathUtil.clamp(manualHoodPos + deltaPerLoop, HOOD_ENC_MIN, HOOD_ENC_MAX);
          ioHood.setHoodPosition(manualHoodPos);
        },
        (interrupted) -> {},
        () -> false,
        this);
  }

  public Command AutoDirectShot(double Postion, double RPM) {
    double clampedRPM = MathUtil.clamp(RPM, -6065, 6065);
    double clampedPostion = MathUtil.clamp(Postion, HOOD_ENC_MIN, HOOD_ENC_MAX);

    return this.runEnd(
        () -> {
          ioMotor.SetRpm(clampedRPM);
          ioHood.setHoodPosition(clampedPostion);
        },
        () -> StowHood());
  }

  // public Command hoodPidFromDashboard() {
  //   return this.run(
  //       () -> {
  //         double targetPosition = MathUtil.clamp(fPercent, HOOD_ENC_MIN, HOOD_ENC_MAX);
  //         ioHood.setHoodPosition(targetPosition);
  //       });
  // }
}
