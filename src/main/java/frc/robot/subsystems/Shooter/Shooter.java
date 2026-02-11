// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShooterIOinputsAutoLogged inputs = new ShooterIOinputsAutoLogged();

  private final ShooterIO io;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter Inputs", inputs);
  }

  public Command RaiseAct() {
    return this.run(() -> io.ExtendAct());
  }

  public Command LowerAct() {
    return this.run(() -> io.RetractAct());
  }
}
