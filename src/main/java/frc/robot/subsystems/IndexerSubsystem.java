// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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

  private SparkMaxPIDController pidController;
  private GenericEntry positionSet;
  private GenericEntry kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private final IndexerIO io;
  private final IndexerIOInputsAutoLoggedv2 inputs = new IndexerIOInputsAutoLoggedv2();

  // Creates a new IntakeSubsystem.
  public IndexerSubsystem() {
    // System.out.println("HI, INDEXER SUBSYSTEM HAS BEEN DECLAIRED");
    // System.out.println("!");
    // System.out.println("!");
    // System.out.println("!");
    // System.out.println("!");
    // System.out.println("!");
    // System.out.println("!");
    // Thread.dumpStack();
    indexerRotationMotor = new CANSparkMax(Constants.INDEXER_ROTATION_CAN_ID, MotorType.kBrushless);
    indexerGrabbingMotor = new CANSparkMax(Constants.INDEXER_GRABBER_CAN_ID, MotorType.kBrushless);
    indexerRotationMotor.restoreFactoryDefaults();
    indexerGrabbingMotor.restoreFactoryDefaults();
    indexerSolenoid = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, 0);
    pidController = indexerRotationMotor.getPIDController();
    pidController.setReference(
        indexerRotationMotor.getEncoder().getPosition(), ControlType.kPosition);

    setupShuffleboard();

    pidController.setP(Constants.INDEXER_ARM_P_VALUE);
    pidController.setI(Constants.INDEXER_ARM_I_VALUE);
    pidController.setD(Constants.INDEXER_ARM_D_VALUE);
    pidController.setIZone(0);
    pidController.setFF(0);
    pidController.setSmartMotionMaxVelocity(Constants.INDEXER_ARM_ROTATE_MAX_SPEED, 0);
    pidController.setOutputRange(-.25, .25);

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
    if (Constants.TUNING_MODE) {
      pidController.setP(kP.getDouble(Constants.INDEXER_ARM_P_VALUE));
      pidController.setI(kI.getDouble(Constants.INDEXER_ARM_I_VALUE));
      pidController.setD(kD.getDouble(Constants.INDEXER_ARM_D_VALUE));
      // pidController.setIZone(kIz.getDouble(0));
      // pidController.setFF(kFF.getDouble(0));
      // pidController.setOutputRange(-.25,25);
      pidController.setReference(positionSet.getDouble(0), ControlType.kSmartMotion);
    }
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

  private void setupShuffleboard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("Indexer");
    tab.addDouble("Pos", () -> indexerRotationMotor.getEncoder().getPosition());
    tab.addDouble("Vel", () -> indexerRotationMotor.getEncoder().getVelocity());
    tab.addDouble(
        "PosFactor", () -> indexerRotationMotor.getEncoder().getPositionConversionFactor());
    tab.addDouble(
        "VelFactor", () -> indexerRotationMotor.getEncoder().getVelocityConversionFactor());
    tab.addDouble("Current (amps)", () -> indexerRotationMotor.getOutputCurrent());
    if (Constants.TUNING_MODE) {
      kP = tab.add("P", .9).withWidget(BuiltInWidgets.kTextView).getEntry();
      kI = tab.add("I", .1).withWidget(BuiltInWidgets.kTextView).getEntry();
      kD = tab.add("D", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
      kIz = tab.add("Iz", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
      kFF = tab.add("FF", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

      kMinOutput = tab.add("Max Output", .25).withWidget(BuiltInWidgets.kTextView).getEntry();
      kMaxOutput = tab.add("Min Output", -.25).withWidget(BuiltInWidgets.kTextView).getEntry();
      positionSet =
          tab.add("Set Position", 0)
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(0, 0)
              .getEntry();
    }
  }
}
