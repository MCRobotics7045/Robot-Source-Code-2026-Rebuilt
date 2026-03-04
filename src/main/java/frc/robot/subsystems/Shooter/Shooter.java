// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShooterIOinputsAutoLogged inputs = new ShooterIOinputsAutoLogged();

  private final ShooterIO ioMotor;
  private final ShooterIO ioACtuator;
  private double fPercent;

  public Shooter(ShooterIO ioMotor, ShooterIO ioACtuator) {
    this.ioMotor = ioMotor;
    this.ioACtuator = ioACtuator;
  }

  @Override
  public void periodic() {
    ioMotor.updateInputs(inputs);
    ioACtuator.updateInputs(inputs);
    Logger.processInputs("Shooter Inputs", inputs);
    fPercent = SmartDashboard.getNumber("Hood Angle", 0);
  }

  public Command Actuator() {
    return this.run(() -> ioACtuator.SetActuatorPercent(fPercent));
  }

  public double ProccesDistanceMotor(double Distance) {
    double targetVolts =
        MathUtil.clamp(ShooterConstants.kDistanceToVoltageMap.get(Distance), -12, 12);
    return targetVolts;
  }

  public double ProccesDistanceActuatorMM(double Distance) {
    double targetLength =
        MathUtil.clamp(ShooterConstants.kDistanceToAngleMap.get(Distance), 0, 100);
    targetLength = (targetLength / 2);
    return targetLength;
  }

  public void FireVoid(double Distance) {
    double LinActPos = ProccesDistanceActuatorMM(Distance);
    double MotorVoltage = ProccesDistanceMotor(Distance);
    ioACtuator.SetActuatorHeightMM(LinActPos);
    ioMotor.RunVoltage(MotorVoltage);
  }

  public Command FireCommand(DoubleSupplier Distance) {
    return this.run(() -> FireVoid(Distance.getAsDouble()));
  }
}
