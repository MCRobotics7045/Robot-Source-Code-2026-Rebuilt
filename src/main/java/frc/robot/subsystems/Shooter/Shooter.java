// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShooterIOinputsAutoLogged inputs = new ShooterIOinputsAutoLogged();

  private final ShooterIO io;
  private final ShooterIO io2;

  public Shooter(ShooterIO io, ShooterIO io2) {
    this.io = io;
    this.io2 = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter Inputs", inputs);
  }

  public Command Actuator(DoubleSupplier Angle) {
    double Angles = Angle.getAsDouble();
    return this.run(() -> io.SetActuatorHeight(Angles));
  }

  public double ProccesDistanceMotor(double Distance) {
    double targetVolts =
        MathUtil.clamp(ShooterConstants.kDistanceToVoltageMap.get(Distance), -12, 12);
    return targetVolts;
  }

  public double ProccesDistanceActuator(double Distance) {
    double targetLength =
        MathUtil.clamp(ShooterConstants.kDistanceToAngleMap.get(Distance), 0, 100);
    targetLength = ((targetLength / 2) * 50);
    return targetLength;
  }

  public void FireVoid(double Distance) {
    double LinActPos = ProccesDistanceActuator(Distance);
    double MotorVoltage = ProccesDistanceMotor(Distance);
    io2.SetActuatorHeight(LinActPos);
    io.RunVoltage(MotorVoltage);
  }

  public Command FireCommand(double Distance) {
    return this.run(() -> FireVoid(Distance));
  }
}
