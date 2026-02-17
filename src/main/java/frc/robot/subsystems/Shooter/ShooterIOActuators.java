// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import frc.robot.util.LinearServo;

/** Add your docs here. */
public class ShooterIOActuators implements ShooterIO {
  private final LinearServo lSideServo = new LinearServo(0, 50, 32);
  private final LinearServo RSideServo = new LinearServo(1, 50, 32);
  double corrected;

  public ShooterIOActuators() {
    corrected = 0;
    RSideServo.setPosition(0);
    lSideServo.setPosition(0);
  }

  public void updateInputs(ShooterIOinputs inputs) {
    lSideServo.updateCurPos();
    RSideServo.updateCurPos();
    inputs.LinearPOSMM = lSideServo.getPosition();
    inputs.RequestedPostionPercent = corrected;
  }

  public void ExtendAct() {
    lSideServo.setPosition(50);
    RSideServo.setPosition(50);
  }

  public void RetractAct() {
    lSideServo.setPosition(0);
    RSideServo.setPosition(0);
  }

  public void SetActuatorHeightMM(double MM) {
    corrected = MM;
    lSideServo.setPosition(MM);
    RSideServo.setPosition(MM);
  }

  public void SetActuatorPercent(double percent) { // 0-100
    double Clamped = MathUtil.clamp(percent, 0, 100);
    corrected = Clamped;
    Clamped = Clamped / 2;
    lSideServo.setPosition(Clamped);
    RSideServo.setPosition(Clamped);
  }
}
