// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX motor;
  // private final TalonFX motorR;
  private final TalonFXConfiguration motorConfig;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  public double SetMotorRPM;
  public double SetMotorVoltage;

  public SimpleMotorFeedforward feedforward;
  public PIDController pid;

  public ShooterIOTalonFX(int lMotorID) {
    motor = new TalonFX(lMotorID);
    // motorR = new TalonFX(rMotorID);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // On-board velocity PID
    motorConfig.Slot0.kV = 12.0 / (6065.0 / 60.0); // volts per RPS (TalonFX velocity is in RPS)
    motorConfig.Slot0.kS = 0.15; // ADDED THIS AFTER AUTO SHOTS
    motorConfig.Slot0.kP = 0.01; // 3/27 CHANGE
    motorConfig.Slot0.kD = 0.0001;

    motor.getConfigurator().apply(motorConfig);

    // feedforward = new SimpleMotorFeedforward(0.15 / 12, 0.00001 / 12);
    // pid = new PIDController(0.0005, 0, 0);
  }

  @Override
  public void updateInputs(ShooterIOinputs inputs) {
    inputs.MotorRPM = motor.getVelocity().getValueAsDouble() * 60;
    inputs.MotorVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.MotorAmp = motor.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void RunVoltage(double Voltage) {
    SetMotorVoltage = Voltage;
    motor.setVoltage(MathUtil.clamp(Voltage, -12.0, 12.0));
  }

  @Override
  public void SetRpm(double Rpm) {
    SetMotorRPM = Rpm;
    motor.setControl(velocityRequest.withVelocity(Rpm / 60.0));
    Logger.recordOutput("Shooter RPM Setpoint", Rpm);
    Logger.recordOutput("RPM Error", SetMotorRPM - motor.getVelocity().getValueAsDouble() * 60);
  }

  @Override
  public void StopMotor() {
    motor.stopMotor();
  }

  public boolean isAtSpeed() {
    double currentRPM = motor.getVelocity().getValueAsDouble() * 60;
    return Math.abs(currentRPM - SetMotorRPM) < 200;
  }
}
