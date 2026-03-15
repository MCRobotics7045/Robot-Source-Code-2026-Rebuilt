// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax Drive;
  private final SparkMax Pos;
  private final SparkMaxConfig config;
  private final SparkClosedLoopController controller;
  private final RelativeEncoder DEncoder;
  private final RelativeEncoder PEncoder;
  private boolean zerDebug = false;
  private double DesiredAngle;

  @SuppressWarnings("removal")
  public IntakeIOSparkMax(int PosMotorID, int DriveMotorID) {
    Drive = new SparkMax(DriveMotorID, MotorType.kBrushless);
    Pos = new SparkMax(PosMotorID, MotorType.kBrushless);
    DEncoder = Drive.getEncoder();
    PEncoder = Pos.getEncoder();
    config = new SparkMaxConfig();
    config
        .smartCurrentLimit(60)
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(
            0.1); // Unsure if I need to make a new Config or can i change it then apply again?
    Drive.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // pOS CONFIG
    config.closedLoop.p(1).i(0.002).d(0.48);
    config
        .closedLoop
        .maxMotion
        .allowedProfileError(0.1)
        .cruiseVelocity(10000)
        
        .maxAcceleration(1000);
    Pos.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    controller = Pos.getClosedLoopController();
  }

  public void updateInputs(IntakeIoinputs inputs) {
    inputs.CurrentAmps = Pos.getOutputCurrent();
    inputs.MotorPos = Units.rotationsToDegrees(PEncoder.getPosition() / 20.0);
    inputs.appliedVolts = Pos.getAppliedOutput() * Pos.getBusVoltage();
    inputs.DAmprege = Drive.getOutputCurrent();
    inputs.DMotorRPM = DEncoder.getVelocity();
    inputs.DappliedVolts = Drive.getAppliedOutput() * Drive.getBusVoltage();
    inputs.DesiredAngle = DesiredAngle;
    inputs.ZeroCommand = zerDebug;
  }

  @Override
  public void setIntakePostion(double angRads) {
    double current = Pos.getOutputCurrent();
    if (current > 35 && Math.abs(Pos.getEncoder().getVelocity()) < 0.1) {
      Pos.stopMotor();
      return;
    }
    controller.setSetpoint(angRads, SparkBase.ControlType.kMAXMotionPositionControl);
    DesiredAngle = angRads;
  }

  public void ZeroIntake() {
    PEncoder.setPosition(0);
    DesiredAngle = 0;
    if (zerDebug) {
      zerDebug = !zerDebug;
    } else {
      zerDebug = !zerDebug;
    }
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
