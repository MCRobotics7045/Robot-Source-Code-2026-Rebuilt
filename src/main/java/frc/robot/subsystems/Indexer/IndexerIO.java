// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public double BeltMotorRPM = 0.0;
    public double BeltappliedVolts = 0.0;
    public double BeltcurrentAmps = 0.0;
    public double StarMotorRPM = 0.0;
    public double StarappliedVolts = 0.0;
    public double StarcurrentAmps = 0.0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void RunIndexerF(double speed) {}

  default void RunIndexerB(double speed) {}

  default void StopIndexer() {}
}
