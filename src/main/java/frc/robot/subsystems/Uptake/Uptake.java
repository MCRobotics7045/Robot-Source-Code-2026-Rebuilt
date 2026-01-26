// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Uptake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Uptake extends SubsystemBase {
  /** Creates a new Uptake. */
  private final UptakeIO io;

  private final UptakeIOInputsAutoLogged inputs = new UptakeIOInputsAutoLogged();

  public Uptake(UptakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Uptake Inputs", inputs);
  }

  public Command runUptake() {
    return this.startEnd(() -> io.runUptake(0.4), () -> io.stopUptake());
  }

  public Command stopUptake() {
    return this.run(() -> io.stopUptake());
  }
}
