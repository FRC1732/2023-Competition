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
import frc.robot.subsystems.io.IndexerIO;
import frc.robot.subsystems.io.IndexerIOInputsAutoLoggedv2;
import org.littletonrobotics.junction.Logger;

public class IndexerSubsystem extends SubsystemBase {
  private CANSparkMax indexerRotationMotor;
  private CANSparkMax indexerGrabbingMotor;
  private Solenoid indexerSolenoid;
  private boolean isOpen = true;

  private final IndexerIO io;
  private final IndexerIOInputsAutoLoggedv2 inputs = new IndexerIOInputsAutoLoggedv2();

  // Creates a new IntakeSubsystem.
  public IndexerSubsystem() {
    indexerRotationMotor = new CANSparkMax(Constants.INDEXER_ROTATION_CAN_ID, MotorType.kBrushless);
    indexerGrabbingMotor = new CANSparkMax(Constants.INDEXER_GRABBER_CAN_ID, MotorType.kBrushless);

    indexerSolenoid = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, 0);

    io =
        new IndexerIO() {

          @Override
          public void updateInputs(IndexerIOInputs inputs) {
            inputs.grabberSpeed = indexerGrabbingMotor.get();
            inputs.grabberCurrent = indexerGrabbingMotor.getOutputCurrent();
            inputs.grabberPosition = indexerGrabbingMotor.getEncoder().getPosition();
            inputs.grabberVelocity = indexerGrabbingMotor.getEncoder().getVelocity();

            inputs.rotationSpeed = indexerRotationMotor.get();
            inputs.rotationCurrent = indexerRotationMotor.getOutputCurrent();
            inputs.rotationPosition = indexerRotationMotor.getEncoder().getPosition();
            inputs.rotationVelocity = indexerRotationMotor.getEncoder().getVelocity();

            inputs.solenoidState = indexerSolenoid.get();
            inputs.isOpen = isOpen();
          }
        };
  }

  protected boolean isOpen() {
    return isOpen;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Indexer", inputs);
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
    isOpen = true;
  }

  public void close() {
    indexerSolenoid.set(false);
    isOpen = false;
  }

  public void eject() {
    if (isOpen) {
      indexerGrabbingMotor.set(1.00);
    } else {
      indexerGrabbingMotor.set(-1.00);
    }
  }

  public double getArmRotation() {
    return indexerRotationMotor.getEncoder().getPosition();
  }
}
