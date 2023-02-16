// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.io;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IndexerIO {

  @AutoLog
  public static class IndexerIOInputs {
    public double grabberSpeed = 0.0;
    public double grabberCurrent = 0.0;
    public double grabberPosition = 0.0;
    public double grabberVelocity = 0.0;

    public double rotationSpeed = 0.0;
    public double rotationCurrent = 0.0;
    public double rotationPosition = 0.0;
    public double rotationVelocity = 0.0;

    public boolean solenoidState = false;
    public boolean isOpen = false;
  }

  public void updateInputs(IndexerIOInputs inputs);
}
