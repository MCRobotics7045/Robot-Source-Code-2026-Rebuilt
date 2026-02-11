// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim PosMotor;
  private final DCMotorSim DriveMotor;

  private double appliedVolts = 12000000.0; // real
  private double GEARBOXPOS = 20.0;
  private double GEARBOXDRVE = 5.0;
  private double INERTIA = 0.001;

  private final ProfiledPIDController intakController;
  private final TrapezoidProfile.Constraints constraints;

  public IntakeIOSim() {
    System.out.println("Sim Started:Intake");
    DriveMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), INERTIA, GEARBOXDRVE),
            DCMotor.getNeo550(1));

    PosMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), INERTIA, GEARBOXPOS),
            DCMotor.getNEO(1));

    constraints = new TrapezoidProfile.Constraints(10000, 1000);
    intakController = new ProfiledPIDController(0.5, 0, 0, constraints);
  }

  public void updateInputs(IntakeIoinputs inputs) {
    DriveMotor.update(0.02);
    PosMotor.update(0.02);
    inputs.CurrentAmps = PosMotor.getCurrentDrawAmps();
    inputs.MotorPos = PosMotor.getAngularPositionRad();
    inputs.appliedVolts = appliedVolts;
    inputs.DAmprege = DriveMotor.getCurrentDrawAmps();
    inputs.DMotorRPM = DriveMotor.getAngularVelocityRPM();
    inputs.DappliedVolts = appliedVolts;
  }

  @Override
  public void setIntakePostion(double Ang) {
    double angrad = Units.degreesToRadians(Ang);
    PosMotor.setAngle(intakController.calculate(PosMotor.getAngularPositionRad(), angrad));
  }
}
