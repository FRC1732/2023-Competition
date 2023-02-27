// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HolderSubsystem extends SubsystemBase {
  /** Creates a new Holder. */
  private Solenoid holderSolenoid;

  private boolean isOpen = true;

  public HolderSubsystem() {
    holderSolenoid =
        new Solenoid(
            Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, Constants.HOLDER_SOLENOID_ID);
  }

  public void open() {
    holderSolenoid.set(true);
    isOpen = true;
  }

  public void close() {
    holderSolenoid.set(false);
    isOpen = false;
  }

  public boolean isOpen() {
    return isOpen;
  }

  public void toggle() {
    if (isOpen) {
      close();
    } else {
      open();
    }
  }
}
