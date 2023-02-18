// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtenderSubsystem extends SubsystemBase {
  private CANSparkMax extenderMotor;
  private DigitalInput extenderMagneticLimitSwitch;

  /** Creates a new IntakeSubsystem. */
  public ExtenderSubsystem() {
    extenderMotor = new CANSparkMax(Constants.EXTENDER_MOTOR_CAN_ID, MotorType.kBrushed);
    extenderMagneticLimitSwitch = new DigitalInput(Constants.EXTENDER_MAGNETIC_LIMIT_SWITCH);
  }

  public void moveIn() {
    extenderMotor.set(-0.1);
  }

  public void stop() {
    extenderMotor.set(0);
  }

  public void moveOut() {
    extenderMotor.set(0.1);
  }

  public boolean getMagSwitch() {
    return extenderMagneticLimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
