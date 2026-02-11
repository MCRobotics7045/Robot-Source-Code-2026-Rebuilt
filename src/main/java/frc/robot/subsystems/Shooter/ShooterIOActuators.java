// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.util.LinearServo;

/** Add your docs here. */
public class ShooterIOActuators implements ShooterIO {
  private final LinearServo lSideServo = new LinearServo(0, 50, 32);

  public ShooterIOActuators() {

    lSideServo.setPosition(0);
  }

  public void updateInputs(ShooterIOinputs inputs) {
    lSideServo.updateCurPos();
    inputs.LienarActuatorPos = lSideServo.getPosition();
  }

  public void ExtendAct() {
    lSideServo.setPosition(45);
  }

  public void RetractAct() {
    lSideServo.setPosition(0);
  }
}
