// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class IndexerIOSparkMax implements IndexerIO {
  private final SparkMax spark;
  private final SparkMaxConfig config;
  private final RelativeEncoder sparkEncoder;
  private final double baseRPM;

  public IndexerIOSparkMax(int MotorID) {
    spark = new SparkMax(MotorID, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.smartCurrentLimit(60).idleMode(IdleMode.kBrake).openLoopRampRate(0.1);
    sparkEncoder = spark.getEncoder();
    baseRPM = 0.3;
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.MotorRPM = sparkEncoder.getVelocity();
    inputs.appliedVolts = spark.getAppliedOutput() * spark.getBusVoltage();
    inputs.currentAmps = spark.getOutputCurrent();
  }

  @Override
  public void RunIndexerF(double speed) {
    // Multi must be from 0.01-5
    spark.set(speed);
  }

  @Override
  public void RunIndexerB(double speed) {
    spark.set(speed);
  }

  @Override
  public void StopIndexer() {
    spark.stopMotor();
  }
}
