package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

public class LinearServo extends Servo {
  double m_speed;
  double m_length;
  double setPos;
  double curPos;
  double lastTime;

  /**
   * Parameters for L16-R Actuonix Linear Actuators
   *
   * @param channel PWM channel used to control the servo
   * @param length max length of the servo [mm]
   * @param speed max speed of the servo [mm/second]
   */
  public LinearServo(int channel, int length, int speed) {
    super(channel);
    // Standard PWM bounds for Actuonix R-series
    setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    m_length = length;
    m_speed = speed;
    lastTime = Timer.getFPGATimestamp();
  }

  /**
   * Sets the target position of the linear actuator.
   *
   * @param setpoint the target position [mm]
   */
  public void setPosition(double setpoint) {
    setPos = MathUtil.clamp(setpoint, 0, m_length);
    // Map 0..length to -1..1 for the setSpeed call
    setSpeed((setPos / m_length * 2) - 1);
  }

  /**
   * Updates the position estimation. Call this in the periodic() method of the subsystem using this
   * servo.
   */
  public void updateCurPos() {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastTime;
    lastTime = currentTime; // Update lastTime to current

    if (curPos > setPos + (m_speed * dt)) {
      curPos -= m_speed * dt;
    } else if (curPos < setPos - (m_speed * dt)) {
      curPos += m_speed * dt;
    } else {
      curPos = setPos;
    }
  }

  @Override
  public double getPosition() {
    return curPos;
  }

  public boolean isFinished() {
    // Use a small epsilon (0.5mm) instead of == for physics/timing stability
    return Math.abs(curPos - setPos) < 0.5;
  }
}
