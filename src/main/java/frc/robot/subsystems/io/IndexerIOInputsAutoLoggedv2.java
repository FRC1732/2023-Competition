// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.io;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public class IndexerIOInputsAutoLoggedv2 extends IndexerIO.IndexerIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("GrabberSpeed", grabberSpeed);
    table.put("GrabberCurrent", grabberCurrent);
    table.put("GrabberPosition", grabberPosition);
    table.put("GrabberVelocity", grabberVelocity);
    table.put("GrabberRotationSpeed", rotationSpeed);
    table.put("GrabberRotationCurrent", rotationCurrent);
    table.put("GrabberRotationPosition", rotationPosition);
    table.put("GrabberRotationVelocity", rotationVelocity);
    table.put("GrabberSolenoidState", solenoidState);
    table.put("GrabberIsOpen", isOpen);
  }

  @Override
  public void fromLog(LogTable table) {
    grabberSpeed = table.getDouble("GrabberSpeed", grabberSpeed);
    grabberCurrent = table.getDouble("GrabberCurrent", grabberCurrent);
    grabberPosition = table.getDouble("GrabberPosition", grabberPosition);
    grabberVelocity = table.getDouble("GrabberVeocity", grabberVelocity);
    rotationSpeed = table.getDouble("GrabberRotationSpeed", rotationSpeed);
    rotationCurrent = table.getDouble("GrabberRotationCurrent", rotationCurrent);
    rotationPosition = table.getDouble("GrabberRotationPosition", rotationPosition);
    rotationVelocity = table.getDouble("GrabberRotationVelocity", rotationVelocity);
    solenoidState = table.getBoolean("GrabberSolenoidState", solenoidState);
    isOpen = table.getBoolean("GrabberIsOpen", isOpen);
  }

  public IndexerIOInputsAutoLoggedv2 clone() {
    IndexerIOInputsAutoLoggedv2 copy = new IndexerIOInputsAutoLoggedv2();
    copy.grabberSpeed = this.grabberSpeed;
    copy.grabberCurrent = this.grabberCurrent;
    copy.grabberPosition = this.grabberPosition;
    copy.grabberVelocity = this.grabberVelocity;
    copy.rotationSpeed = this.rotationSpeed;
    copy.rotationCurrent = this.rotationCurrent;
    copy.rotationPosition = this.rotationPosition;
    copy.rotationVelocity = this.rotationVelocity;
    copy.solenoidState = this.solenoidState;
    copy.isOpen = this.isOpen;
    return copy;
  }
}
