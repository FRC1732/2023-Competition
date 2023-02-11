// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax motorController;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    motorController = new CANSparkMax(Constants.INTAKE_CAN_ID, MotorType.kBrushed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void on() {
    motorController.set(-0.375);
  }

  public void off() {
    motorController.set(0);
  }

  public void eject() {
    motorController.set(1.00);
  }
}
