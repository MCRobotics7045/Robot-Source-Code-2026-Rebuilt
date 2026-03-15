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

public class ShooterIOHoodMotor implements ShooterIO {

  public static final int HOOD_CAN_ID = 35;
  public static final double ENCODER_MIN = 0.0;
  public static final double ENCODER_MAX = 1.2;

  private final SparkMax motor;
  private final RelativeEncoder encoder;

  @SuppressWarnings("removal")
  public ShooterIOHoodMotor() {
    motor = new SparkMax(HOOD_CAN_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(30).idleMode(IdleMode.kBrake).openLoopRampRate(0.1);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(ShooterIOinputs inputs) {
    inputs.HoodAngle = MathUtil.clamp(encoder.getPosition(), ENCODER_MIN, ENCODER_MAX);
  }

  @Override
  public void setHoodVoltage(double volts) {
    double pos = MathUtil.clamp(encoder.getPosition(), ENCODER_MIN, ENCODER_MAX);
    if ((pos <= ENCODER_MIN && volts < 0) || (pos >= ENCODER_MAX && volts > 0)) {
      motor.stopMotor();
      return;
    }
    motor.setVoltage(volts);
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
