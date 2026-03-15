// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  
  private static final double HOOD_kP = 1;
  private static final double HOOD_kI = 0.0;
  private static final double HOOD_kD = 0.0;

  private static final double HOOD_kG = 0.5;

  private static final double HOOD_TOLERANCE = 0.02; // rotations

  private static final double HOOD_ENC_MIN = ShooterIOHoodMotor.ENCODER_MIN;
  private static final double HOOD_ENC_MAX = ShooterIOHoodMotor.ENCODER_MAX;

  private final ShooterIOinputsAutoLogged inputs = new ShooterIOinputsAutoLogged();

  private final ShooterIO ioMotor;
  private final ShooterIO ioActuator;
  private final ShooterIO ioHood;

  // private double kP, kI, kD;

  private final PIDController hoodPID;
  private double fPercent;

  public Shooter(ShooterIO ioMotor, ShooterIO ioActuator, ShooterIO ioHood) {
    this.ioMotor = ioMotor;
    this.ioActuator = ioActuator;
    this.ioHood = ioHood;

    hoodPID = new PIDController(HOOD_kP, HOOD_kI, HOOD_kD);
    hoodPID.setTolerance(HOOD_TOLERANCE);
  }

  @Override
  public void periodic() {
    ioMotor.updateInputs(inputs);
    ioActuator.updateInputs(inputs);
    ioHood.updateInputs(inputs);
    Logger.processInputs("Shooter Inputs", inputs);

    fPercent = SmartDashboard.getNumber("Hood Angle", 0);
    // kP = SmartDashboard.getNumber("Kp", 0);
    // kI = SmartDashboard.getNumber("Ki", 0);
    // kD = SmartDashboard.getNumber("Kd", 0);

    // hoodPID.setP(kP);
    // hoodPID.setI(kI);
    // hoodPID.setD(kD);
    SmartDashboard.putNumber("Hood Position (enc)", inputs.HoodAngle);
    SmartDashboard.putBoolean("Hood At Setpoint", hoodPID.atSetpoint());
  }

  public Command Actuator() {
    return this.run(() -> ioActuator.SetActuatorPercent(fPercent));
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
    ioActuator.SetActuatorHeightMM(LinActPos);
    ioMotor.RunVoltage(MotorVoltage);
  }

  public Command FireCommand(DoubleSupplier Distance) {
    return this.run(() -> FireVoid(Distance.getAsDouble()));
  }

  public Command FireBlankCommand(double Voltage) {
    return this.startEnd(() -> ioMotor.RunVoltage(Voltage), () -> ioMotor.StopMotor());
  }

  public Command hoodStop() {
    return this.runOnce(() -> ioHood.StopMotor());
  }

  /**
   * @param volts voltage to apply (positive = toward ENCODER_MAX)
   */
  public Command hoodRunVoltage(double volts) {
    return this.startEnd(() -> ioHood.setHoodVoltage(volts), () -> ioHood.StopMotor());
  }

  public Command hoodPidToPosition(double targetPosition) {
    double clampedTarget = MathUtil.clamp(targetPosition, HOOD_ENC_MIN, HOOD_ENC_MAX);
    return this.run(
        () -> {
          double pidOutput = hoodPID.calculate(inputs.HoodAngle, clampedTarget);
          double output = MathUtil.clamp(pidOutput + HOOD_kG, -12.0, 12.0);
          ioHood.setHoodVoltage(output);
        });
  }

  public Command hoodPidFromDashboard() {
    return this.run(
        () -> {
          // 0–100  →  0–1.2 rotations
          double targetPosition = MathUtil.clamp(fPercent, HOOD_ENC_MIN, HOOD_ENC_MAX);
          double pidOutput = hoodPID.calculate(inputs.HoodAngle, targetPosition);
          double output = MathUtil.clamp(pidOutput + HOOD_kG, -12.0, 12.0);
          ioHood.setHoodVoltage(output);
        });
  }
}
