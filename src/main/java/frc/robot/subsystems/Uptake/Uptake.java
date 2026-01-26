// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Uptake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Uptake extends SubsystemBase {
  /** Creates a new Uptake. */
  private final UptakeIO io;

  public Uptake(UptakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double UptakeSetSpeed () {
    return 0;
  }
  
  public Command runUptake() {
    return this.startEnd(null, null);
  }

  public Command stopUptake() {
    return this.startEnd(null, null);
  }
}
