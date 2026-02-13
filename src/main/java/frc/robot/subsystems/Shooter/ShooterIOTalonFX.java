// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX motorL;
  private final TalonFX motorR;
  private final TalonFXConfiguration motorConfig;

  public ShooterIOTalonFX(int lMotorID, int rMotorID) {
    motorL = new TalonFX(lMotorID);
    motorR = new TalonFX(rMotorID);

    motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.CurrentLimits.StatorCurrentLimit = 20.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    ///////////////////// Check if this is right////////////////////Its not. Thiswill spin them in
    // the same direction.
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ///////////////////////////////////////////////////////////////

    motorL.getConfigurator().apply(motorConfig);
    motorR.getConfigurator().apply(motorConfig);
  }

  @Override
  public void updateInputs(ShooterIOinputs inputs) {
    inputs.lMotorRPM = motorL.getVelocity().getValueAsDouble();
    inputs.rMotorRPM = motorR.getVelocity().getValueAsDouble();
    inputs.lMotorVolts = motorL.getSupplyVoltage().getValueAsDouble();
    inputs.lMotorAmp = motorL.getSupplyCurrent().getValueAsDouble();
    inputs.rMotorVolts = motorR.getSupplyVoltage().getValueAsDouble();
    inputs.rMotorAmp = motorR.getSupplyCurrent().getValueAsDouble();
  }

  @Override
  public void RunVoltage(double Voltage) {
    motorL.setVoltage(MathUtil.clamp(Voltage, -12.0, 12.0));
    motorR.setVoltage(MathUtil.clamp(-Voltage, -12.0, 12.0));
  }

  @Override
  public void StopMotor() {
    motorL.stopMotor();
    motorR.stopMotor();
  }
}
