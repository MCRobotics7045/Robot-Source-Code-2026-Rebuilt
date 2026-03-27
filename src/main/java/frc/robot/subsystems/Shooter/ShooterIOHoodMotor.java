// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.Logger;

public class ShooterIOHoodMotor implements ShooterIO {

  public static final int HOOD_CAN_ID = 35;
  public static final double ENCODER_MIN = 0.0;
  public static final double ENCODER_MAX = 1.2;

  private static final double TOLERANCE = 0.2; // rotations

  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pid;
  private double appliedVolts = 0.0;

  @SuppressWarnings("removal")
  public ShooterIOHoodMotor() {
    motor = new SparkMax(HOOD_CAN_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40).idleMode(IdleMode.kBrake).openLoopRampRate(0.1);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

    pid = new PIDController(3, 0, 0);
    pid.setTolerance(0.03);

    /**
     * STEP 1: Find Gravity Feedforward (P=I=D all at 0) Set gravityFeedforward = 0.0 Command hood
     * to some position (like 0.5) Slowly increase feedforward (0.2, 0.4, 0.6, 0.8...) Stop when the
     * hood stops sliding back down after you release it manually That's your gravity feedforward
     * value → write it down
     *
     * <p>STEP 2: Add P Gain (I and D stay 0) Keep gravityFeedforward at the value from Step 1 Set P
     * to 2.0 (starting point) Command hood to setpoint (0.2) and watch logs If hood oscillates:
     * decrease P to 1.5 If hood doesn't move to setpoint: increase P to 3.0, 4.0, etc. Sweet spot:
     * settles smoothly to setpoint within ~1 second Write down your P value
     *
     * <p>STEP 3: Add I Gain (if needed) Keep your P and feedforward Start with I = 0.5 Only if P
     * alone leaves you slightly off target, increase I slightly Too much I = oscillation Usually P
     * alone is enough for position control Write down your I value
     *
     * <p>What to watch in logs: Hood/ErrorRotations → should go to ~0 Hood/TotalOutputVolts →
     * should be feedforward + small PID corrections Hood/AtSetpoint → should be true when settled .
     */
  }

  @Override
  public void updateInputs(ShooterIOinputs inputs) {
    inputs.MotorHoodAngle = encoder.getPosition();
    inputs.hoodAppliedVolts = appliedVolts;
    inputs.hoodCurrentAmps = motor.getOutputCurrent();
  }

  @Override
  public void setHoodPosition(double targetRotations) {

    double pos = encoder.getPosition();
    double pidOutput = pid.calculate(pos, targetRotations);

    double gravityFeedforward = 0.25;

    double totalOutput = gravityFeedforward + pidOutput;
    double clampedOutput = MathUtil.clamp(totalOutput, -12.0, 12.0);

    Logger.recordOutput("Hood/TargetRotations", targetRotations);
    Logger.recordOutput("Hood/CurrentRotations", pos);
    Logger.recordOutput("Hood/ErrorRotations", targetRotations - pos);
    Logger.recordOutput("Hood/PIDOutput", pidOutput);
    Logger.recordOutput("Hood/GravityFeedforward", gravityFeedforward);
    Logger.recordOutput("Hood/TotalOutputVolts", totalOutput);
    Logger.recordOutput("Hood/ClampedOutputVolts", clampedOutput);
    Logger.recordOutput("Hood/Tempetaure", motor.getMotorTemperature());
    Logger.recordOutput("Hood/AtSetpoint", pid.atSetpoint());

    motor.setVoltage(clampedOutput);
    appliedVolts = clampedOutput;
  }

  @Override
  public boolean isHoodAtSetpoint() {
    return pid.atSetpoint();
  }

  @Override
  public void resetHoodEncoder() {
    encoder.setPosition(0.0);
  }

  @Override
  public void StopMotor() {
    motor.stopMotor();
  }
}
