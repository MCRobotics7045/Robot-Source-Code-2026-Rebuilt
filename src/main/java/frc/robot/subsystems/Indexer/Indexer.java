// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  private final IndexerIO io;
  private final IndexerIO ioStar;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO io, IndexerIO ioStar) {
    this.io = io;
    this.ioStar = ioStar;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    ioStar.updateInputs(inputs);
    Logger.processInputs("Indexer Inputs", inputs);
  }

  public Command RunIndexerF(double speedMulti) {
    return this.startEnd(() -> io.RunIndexerF(0.3 * speedMulti), () -> io.StopIndexer());
  }

  public Command RunIndexerB(double speedMulti) {
    return this.startEnd(() -> io.RunIndexerB(0.3 * speedMulti), () -> io.StopIndexer());
  }

  public Command RunStarWheels(double speedMulti) {
    return this.startEnd(() -> ioStar.RunIndexerF(-1), () -> ioStar.StopIndexer());
  }

  public Command RunBothIndexer(double speedMulti) {
    return this.startEnd(
        () -> {
          io.RunIndexerF(0.3 * speedMulti);
          ioStar.RunIndexerF(-1);
        },
        () -> {
          io.StopIndexer();
          ioStar.StopIndexer();
        });
  }

  public Command StopIndexer() {
    return this.run(() -> io.StopIndexer());
  }

  public Command StopStar() {
    return this.run(() -> ioStar.StopIndexer());
  }
}
