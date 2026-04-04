// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOTalonFXPos implements IntakeIO {
  private final TalonFX Pos;
  private final SparkMax Drive;
  private final RelativeEncoder DEncoder;
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private boolean zerDebug = false;
  private double DesiredAngle;

  @SuppressWarnings("removal")
  public IntakeIOTalonFXPos(int PosMotorID, int DriveMotorID) {
    Pos = new TalonFX(PosMotorID);
    Drive = new SparkMax(DriveMotorID, MotorType.kBrushless);
    DEncoder = Drive.getEncoder();

    // Configure position motor (Falcon 500 / TalonFX)
    TalonFXConfiguration posConfig = new TalonFXConfiguration();
    posConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    posConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    posConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;
    posConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
    posConfig.CurrentLimits.StatorCurrentLimit = 35.0;
    posConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // PID gains
    posConfig.Slot0.kP = 1.0;
    posConfig.Slot0.kI = 0.0;
    posConfig.Slot0.kD = 0.2;
    posConfig.Slot0.kS = 0.6; // FF static voltage
    posConfig.Slot0.kV = 0.0;
    // Motion Magic settings
    posConfig.MotionMagic.MotionMagicCruiseVelocity = 5;
    posConfig.MotionMagic.MotionMagicAcceleration = 2;
    posConfig.MotionMagic.MotionMagicJerk = 20;
    posConfig.MotionMagic.MotionMagicExpo_kV = 0.12;
    posConfig.MotionMagic.MotionMagicExpo_kA = 0.10;
    Pos.getConfigurator().apply(posConfig);
    Pos.setPosition(0);

    // Configure drive motor (SparkMax, unchanged)
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.smartCurrentLimit(80).idleMode(IdleMode.kBrake).openLoopRampRate(0.1);
    Drive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIoinputs inputs) {
    inputs.CurrentAmps = Pos.getStatorCurrent().getValueAsDouble();
    inputs.MotorPos = Pos.getPosition().getValueAsDouble();
    inputs.appliedVolts = Pos.getMotorVoltage().getValueAsDouble();
    inputs.DAmprege = Drive.getOutputCurrent();
    inputs.DMotorRPM = DEncoder.getVelocity();
    inputs.DappliedVolts = Drive.getAppliedOutput() * Drive.getBusVoltage();
    inputs.DesiredAngle = DesiredAngle;
    inputs.ZeroCommand = zerDebug;
  }

  @Override
  public void setIntakePostion(double angRots) {
    double current = Pos.getStatorCurrent().getValueAsDouble();
    double velocity = Pos.getVelocity().getValueAsDouble();
    if (current > 35 && Math.abs(velocity) < 0.1) {
      Pos.stopMotor();
      return;
    }
    Pos.setControl(motionMagicRequest.withPosition(angRots));
    DesiredAngle = angRots;
  }

  @Override
  public void ZeroIntake() {
    Pos.setPosition(0);
    DesiredAngle = 0;
    zerDebug = !zerDebug;
  }

  @Override
  public void runIntakeD(double speed) {
    Drive.set(speed);
  }

  @Override
  public void stopIntakePos() {
    Pos.stopMotor();
  }

  @Override
  public void stopIntakeD() {
    Drive.stopMotor();
  }
}
