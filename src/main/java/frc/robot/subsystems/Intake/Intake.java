// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static frc.robot.Constants.MotorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;
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
  private boolean isDeployed = false;
  private double manualPos = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake Inputs", inputs);

    Logger.recordOutput("Intake Boolean", isDeployed);
    offset = Units.degreesToRadians(inputs.MotorPos) + Units.degreesToRadians(35);
    hingeRotation = new Rotation3d(0, offset, 0);
    armPose = new Pose3d(hingeLocation, hingeRotation);
    mechPub.set(armPose);
  }

  public Command ToggleIntakeComand(double angle, double angle2) {
    return this.runOnce(
        () -> {
          if (isDeployed) {
            io.setIntakePostion(angle);
          } else {
            io.setIntakePostion(angle2);
          }
          isDeployed = !isDeployed;
        });
  }

  public Command SetIntakeCommand(double Angle) {
    return this.runOnce(() -> io.setIntakePostion(Angle));
  }

  public Command ReturnIntake() {
    return this.run(
        () -> {
          isDeployed = false;
          io.setIntakePostion(IntakeStowed);
        });
  }

  public Command IntakeCommand(double OutAngle, double IntakeSpeed) {
    return Commands.defer(
        () -> {
          if (isDeployed) {
            return RunIntakeShaft(IntakeSpeed);
          } else {
            isDeployed = true;
            return new SequentialCommandGroup(
                run(() -> io.setIntakePostion(OutAngle)).withTimeout(1),
                RunIntakeShaft(IntakeSpeed));
          }
        },
        Set.of(this));
  }

  public Command ShutterBalls(double MaxShutter) {
    return this.run(() -> io.setIntakePostion(MaxShutter));
  }

  public Command RunIntakeShaft(double speed) {
    return this.startEnd(() -> io.runIntakeD(speed), () -> io.stopIntakeD());
  }

  public Command StopIntakeShaft() {
    return this.runOnce(() -> io.stopIntakeD());
  }

  public Command ZeroIntake() {
    return this.runOnce(() -> io.ZeroIntake());
  }

  public Command ManualIntakeAdjust(double deltaPerLoop) {
    return new FunctionalCommand(
        () -> manualPos = inputs.MotorPos,
        () -> {
          manualPos = MathUtil.clamp(manualPos + deltaPerLoop, IntakeStowed, IntakeCollect);
          io.setIntakePostion(manualPos);
        },
        (interrupted) -> {},
        () -> false,
        this);
  }

  public Command RetractWithRollers(double angle, double speed) {
    return this.startEnd(
        () -> {
          io.setIntakePostion(angle);
          io.runIntakeD(speed);
        },
        () -> {
          io.setIntakePostion(0);
          io.stopIntakeD();
        });
  }
}
