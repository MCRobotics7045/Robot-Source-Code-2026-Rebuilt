// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO io;

  private final IntakeIoinputsAutoLogged inputs = new IntakeIoinputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake Inputs", inputs);
  }

  public Command SetIntakeAngle(double Angle) {
    return this.run(() -> io.setIntakePostion(Angle));
  }

  public Command RunIntakeShaft(double speed) {
    return this.startEnd(() -> io.runIntakeD(speed), () -> io.stopIntakeD());
  }

  public Command StopIntakeShaft() {
    return this.startEnd(() -> io.stopIntakeD(), () -> io.stopIntakeD());
  }
}
