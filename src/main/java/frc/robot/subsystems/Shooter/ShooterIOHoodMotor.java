// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.Logger;

public class ShooterIOHoodMotor implements ShooterIO {

  public static final int HOOD_CAN_ID = 35;
  public static final double ENCODER_MIN = 0.0;
  public static final double ENCODER_MAX = 1.2;

  private static final double kP = 4.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kG = 0.5; // Tune
  private static final double TOLERANCE = 0.02; // rotations

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid;
  private double appliedVolts = 0.0;

  @SuppressWarnings("removal")
  public ShooterIOHoodMotor() {
    motor = new SparkMax(HOOD_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(30).idleMode(IdleMode.kBrake).openLoopRampRate(0.1);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(TOLERANCE);
  }

  @Override
  public void updateInputs(ShooterIOinputs inputs) {
    inputs.MotorHoodAngle = encoder.getPosition();
    inputs.hoodAppliedVolts = appliedVolts;
    inputs.hoodCurrentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setHoodPosition(double targetRotations) {
    double pos = encoder.getPosition();
    double pidOutput = pid.calculate(pos, targetRotations);
    double clampedOutput = MathUtil.clamp(pidOutput + kG, -12.0, 12.0);
    boolean softLimitHit =
        (pos <= ENCODER_MIN && clampedOutput < 0) || (pos >= ENCODER_MAX && clampedOutput > 0);

    Logger.recordOutput("Hood/TargetRotations", targetRotations);
    Logger.recordOutput("Hood/CurrentRotations", pos);
    Logger.recordOutput("Hood/ErrorRotations", targetRotations - pos);
    Logger.recordOutput("Hood/RawPIDOutput", pidOutput);
    Logger.recordOutput("Hood/ClampedOutputVolts", clampedOutput);
    Logger.recordOutput("Hood/SoftLimitHit", softLimitHit);
    Logger.recordOutput("Hood/AtSetpoint", pid.atSetpoint());

    if (softLimitHit) {
      motor.stopMotor();
      appliedVolts = 0.0;
      return;
    }
    motor.setVoltage(clampedOutput);
    appliedVolts = clampedOutput;
  }

  @Override
  public boolean isHoodAtSetpoint() {
    return pid.atSetpoint();
  }

  @Override
  public void resetHoodEncoder() {
    encoder.setPosition(0.0);
  }

  @Override
  public void StopMotor() {
    motor.stopMotor();
  }
}
