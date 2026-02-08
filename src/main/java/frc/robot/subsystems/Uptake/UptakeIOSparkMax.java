// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Uptake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class UptakeIOSparkMax implements UptakeIO {
  private final SparkMax spark;
  private final SparkMaxConfig config;
  private final RelativeEncoder sparkEncoder;

  @SuppressWarnings("removal")
  public UptakeIOSparkMax(int MotorID) {
    spark = new SparkMax(MotorID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.smartCurrentLimit(60).idleMode(IdleMode.kBrake).openLoopRampRate(0.1);
    spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    sparkEncoder = spark.getEncoder();
  }

  public void updateInputs(UptakeIOInputs inputs) {
    inputs.MotorRPM = sparkEncoder.getVelocity();
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();
  }

  @Override
  public void runUptake(double SetSpeed) {
    spark.set(SetSpeed);
  }

  @Override
  public void stopUptake() {
    spark.stopMotor();
  }
}
