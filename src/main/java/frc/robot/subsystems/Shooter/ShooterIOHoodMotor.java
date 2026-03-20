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

public class ShooterIOHoodMotor implements ShooterIO {

  public static final int HOOD_CAN_ID = 35;
  public static final double ENCODER_MIN = 0.0;
  public static final double ENCODER_MAX = 1.2;

  private static final double kP = 1.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double TOLERANCE = 0.02; // rotations

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid;

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
  }

  @Override
  public void setHoodPosition(double targetRotations) {
    double pos = encoder.getPosition();
    double pidOutput = pid.calculate(pos, targetRotations);
    double output = MathUtil.clamp(pidOutput, -12.0, 12.0);
    if ((pos <= ENCODER_MIN && output < 0) || (pos >= ENCODER_MAX && output > 0)) {
      motor.stopMotor();
      return;
    }
    motor.setVoltage(output);
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
