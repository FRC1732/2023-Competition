// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  private CANSparkMax indexerRotationMotor;
  private CANSparkMax indexerGrabbingMotor;
  private Solenoid indexerSolenoid;
  boolean IsOpen = true;

  // Creates a new IntakeSubsystem.
  public IndexerSubsystem() {
    indexerRotationMotor = new CANSparkMax(Constants.INDEXER_ROTATION_CAN_ID, MotorType.kBrushless);
    indexerGrabbingMotor = new CANSparkMax(Constants.INDEXER_GRABBER_CAN_ID, MotorType.kBrushless);

    indexerSolenoid = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void grabberOn() {
    indexerGrabbingMotor.set(0.25);
  }

  public void grabberOff() {
    indexerGrabbingMotor.set(0);
  }

  public void grabberEject() {
    indexerGrabbingMotor.set(-0.25);
  }

  public void rotateUp() {
    indexerRotationMotor.set(0.1);
  }

  public void rotateDown() {
    indexerRotationMotor.set(-0.1);
  }

  public void rotateOff() {
    indexerRotationMotor.set(0);
  }

  public void open() {
    indexerSolenoid.set(true);
    IsOpen = true;
  }

  public void close() {
    indexerSolenoid.set(false);
    IsOpen = false;
  }

  public void eject() {
    if (IsOpen) {
      indexerGrabbingMotor.set(1.00);
    } else {
      indexerGrabbingMotor.set(-1.00);
    }
  }
}
