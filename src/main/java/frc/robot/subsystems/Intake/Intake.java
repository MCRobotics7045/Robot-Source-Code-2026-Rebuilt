// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;

  private final IntakeIoinputsAutoLogged inputs = new IntakeIoinputsAutoLogged();

  StructPublisher<Pose3d> mechPub =
      NetworkTableInstance.getDefault().getStructTopic("Intake", Pose3d.struct).publish();

  Translation3d hingeLocation = new Translation3d(0.254, 0.0, 0.184);
  double offset;
  Rotation3d hingeRotation;
  Pose3d armPose;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake Inputs", inputs);
    offset = inputs.MotorPos + Units.degreesToRadians(35);
    hingeRotation = new Rotation3d(0, offset, 0);
    armPose = new Pose3d(hingeLocation, hingeRotation);
    mechPub.set(armPose);
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
