// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;

public class LEDSubsystem extends SubsystemBase {
  public static AddressableLED LeftSide;
  public static AddressableLED RightSide;

  public static AddressableLEDBuffer LEDbuffer;
  public boolean isLEDrunning = false;
  private final LEDPattern m_rainbow;
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  private static final int CENTER_LED = 23;
  private static final int RIGHT_START = 0;
  private static final int RIGHT_LENGTH = CENTER_LED; // 29
  private static final int LEFT_START = CENTER_LED + 1; // 30
  private static final int LEFT_LENGTH = 60 - LEFT_START; // 30

  private final AddressableLEDBuffer m_halfBuffer = new AddressableLEDBuffer(RIGHT_LENGTH);

  private static final LEDPattern base =
      LEDPattern.steps(Map.of(0, Color.kWhite, 0.5, Color.kBlue));
  private static final LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));

  private static final LEDPattern SlowBase =
      LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kDarkOrange));
  private static final LEDPattern SLOW_PATTERN =
      SlowBase.scrollAtRelativeSpeed(Percent.per(Second).of(10));

  private static final LEDPattern BreathBase =
      LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
  private static final LEDPattern BREATH_LED_PATTERN = BreathBase.breathe(Seconds.of(1));

  private static final LEDPattern Blink = LEDPattern.solid(Color.kRed);
  private static final LEDPattern BlinkBad = Blink.blink(Seconds.of(1));

  private static final LEDPattern BlinkG = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern BlinkGood = BlinkG.blink(Seconds.of(1));

  private int m_laserStep = 0;
  private int m_laserTick = 0;
  private static final double LASER_TICKS_PER_STEP = 0.5;

  public LEDSubsystem() {

    LeftSide = new AddressableLED(0);
    LEDbuffer = new AddressableLEDBuffer(60);
    LeftSide.setLength(LEDbuffer.getLength());
    LeftSide.setData(LEDbuffer);
    LeftSide.start();

    m_rainbow = LEDPattern.rainbow(255, 128);
    // m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    setDefaultCommand(PulseColor(getAllianceColor(), 2));
  }

  @Override
  public void periodic() {
    LeftSide.setData(LEDbuffer);
  }

  private void applyMirrored(LEDPattern ledPattern) {
    ledPattern.applyTo(m_halfBuffer);

    // Right channel — direct copy
    for (int i = 0; i < RIGHT_LENGTH; i++) {
      LEDbuffer.setLED(RIGHT_START + i, m_halfBuffer.getLED(i));
    }
    LEDbuffer.setLED(CENTER_LED, Color.kBlack);

    for (int i = 0; i < LEFT_LENGTH; i++) {
      int src = Math.max(0, RIGHT_LENGTH - 1 - i);
      LEDbuffer.setLED(LEFT_START + i, m_halfBuffer.getLED(src));
    }
  }

  public Command runPattern(LEDPattern ledPattern) {
    return run(() -> applyMirrored(ledPattern));
  }

  public Command PulseCrusader() {
    return run(() -> applyMirrored(pattern));
  }

  public Command BlinkBadC() {
    return run(() -> applyMirrored(BlinkBad));
  }

  public Command BlinkGoodC() {
    return run(() -> applyMirrored(BlinkGood));
  }

  public Command LEDShooterPulse() {
    return run(() -> applyLaser(Color.kBlue, 5));
  }

  private void applyLaser(Color color, int laserLength) {

    for (int i = 0; i < LEDbuffer.getLength(); i++) {
      LEDbuffer.setLED(i, Color.kBlack);
    }

    for (int t = 0; t < laserLength; t++) {
      int rightIdx = (CENTER_LED - 1) - m_laserStep + t;
      if (rightIdx >= RIGHT_START && rightIdx < CENTER_LED) {
        LEDbuffer.setLED(rightIdx, color);
      }
    }

    for (int t = 0; t < laserLength; t++) {
      int leftIdx = LEFT_START + m_laserStep - t;
      if (leftIdx >= LEFT_START && leftIdx < LEFT_START + LEFT_LENGTH) {
        LEDbuffer.setLED(leftIdx, color);
      }
    }

    m_laserTick++;
    if (m_laserTick >= LASER_TICKS_PER_STEP) {
      m_laserTick = 0;
      m_laserStep++;
      if (m_laserStep >= Math.max(RIGHT_LENGTH, LEFT_LENGTH)) {
        m_laserStep = 0;
      }
    }
  }

  public Command LaserShot(Color color, int laserLength) {
    return runOnce(
            () -> {
              m_laserStep = 0;
              m_laserTick = 0;
            })
        .andThen(run(() -> applyLaser(color, laserLength)));
  }

  public Color getAllianceColor() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red ? Color.kRed : Color.kBlue;
    }
    return Color.kWhite;
  }

  public Command SetColor(Color color) {
    return run(() -> applyMirrored(LEDPattern.solid(color)));
  }

  // Smoothly fades the color on and off. periodSeconds = time for one full fade cycle.
  public Command PulseColor(Color color, double periodSeconds) {
    LEDPattern pulse = LEDPattern.solid(color).breathe(Seconds.of(periodSeconds));
    return run(() -> applyMirrored(pulse));
  }
}
