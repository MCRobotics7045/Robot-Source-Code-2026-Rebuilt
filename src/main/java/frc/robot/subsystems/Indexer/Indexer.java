// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final IndexerIO io;

  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Indexer Inputs", inputs);
  }

  public Command RunIndexerF(double speedMulti) {
    return this.startEnd(() -> io.RunIndexerF(0.3 * speedMulti), () -> io.StopIndexer());
  }

  public Command RunIndexerB(double speedMulti) {
    return this.startEnd(() -> io.RunIndexerB(0.3 * speedMulti), () -> io.StopIndexer());
  }

  public Command StopIndexer() {
    return this.run(() -> io.StopIndexer());
  }
}
