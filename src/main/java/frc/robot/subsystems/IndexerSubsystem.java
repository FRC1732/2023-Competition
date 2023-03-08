// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.subsystems.io.IndexerIO;
import frc.robot.subsystems.io.IndexerIOInputsAutoLoggedv2;

public class IndexerSubsystem extends SubsystemBase {
  private CANSparkMax indexerRotationMotor;
  private CANSparkMax indexerGrabbingMotor;
  private Solenoid indexerSolenoid;
  private boolean isOpen = false;

  private SparkMaxPIDController pidController;
  private GenericEntry positionSet;
  private GenericEntry kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, kMaxVelocity, kMaxAccel;
  private double preP,
      preI,
      preD,
      preIz,
      preFF,
      preMaxOutput,
      preMinOutput,
      preMaxVelocity,
      preMaxAccel;
  private double motorSpeed;
  private double setpoint;
  private GenericEntry motorSpeedEntry;
  private boolean brakeMode;
  private double prevSetpoint = 0;

  private final IndexerIO io;
  private final IndexerIOInputsAutoLoggedv2 inputs = new IndexerIOInputsAutoLoggedv2();

  // Creates a new IntakeSubsystem.
  public IndexerSubsystem() {
    indexerRotationMotor = new CANSparkMax(Constants.INDEXER_ROTATION_CAN_ID, MotorType.kBrushless);
    indexerGrabbingMotor = new CANSparkMax(Constants.INDEXER_GRABBER_CAN_ID, MotorType.kBrushless);
    indexerRotationMotor.restoreFactoryDefaults();
    indexerRotationMotor
        .getEncoder()
        .setPositionConversionFactor(Constants.INDEXER_POSITION_CONVERSION_FACTOR);
    indexerGrabbingMotor.restoreFactoryDefaults();
    indexerSolenoid = new Solenoid(Constants.CAN_PNEUMATIC_ID, PneumaticsModuleType.REVPH, 0);
    pidController = indexerRotationMotor.getPIDController();
    pidController.setReference(
        indexerRotationMotor.getEncoder().getPosition(), ControlType.kPosition);
    prevSetpoint = indexerRotationMotor.getEncoder().getPosition();
    indexerRotationMotor.getEncoder().setPosition(Constants.INDEXER_STARTING_POSITION);
    setupShuffleboard();
    pidController.setP(Constants.INDEXER_ARM_P_VALUE);
    pidController.setI(Constants.INDEXER_ARM_I_VALUE);
    pidController.setD(Constants.INDEXER_ARM_D_VALUE);
    pidController.setIZone(0);
    pidController.setFF(0);
    pidController.setSmartMotionMaxVelocity(Constants.INDEXER_ARM_ROTATE_MAX_SPEED, 0);
    pidController.setSmartMotionMaxAccel(Constants.INDEXER_ARM_ROTATE_MAX_ACCELERATION, 0);
    pidController.setOutputRange(
        Constants.INDEXER_ARM_PID_MIN_OUTPUT, Constants.INDEXER_ARM_PID_MAX_OUTPUT);
    pidController.setSmartMotionAllowedClosedLoopError(0, 0);
    setpoint = Constants.INDEXER_STARTING_POSITION;
    motorSpeed = .015;
    indexerRotationMotor.burnFlash();
    indexerGrabbingMotor.burnFlash();
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

  public void setBrakeMode() {
    indexerRotationMotor.setIdleMode(IdleMode.kBrake);
    indexerGrabbingMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    indexerRotationMotor.setIdleMode(IdleMode.kCoast);
    indexerGrabbingMotor.setIdleMode(IdleMode.kCoast);
  }

  public boolean isAtSetpoint() {
    boolean temp =
        Math.abs(indexerRotationMotor.getEncoder().getPosition() - setpoint)
            < Constants.INDEXER_ARM_DEADBAND;
    if (temp) {
      System.out.println("Indexer at setpoint");
    }
    return temp;
  }

  public boolean isOpen() {
    return isOpen;
  }

  public boolean hasPiece() {
    return indexerGrabbingMotor.getOutputCurrent() > Constants.INDEXER_PIECE_DETECTION_CURRENT;
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled() && !brakeMode) {
      brakeMode = true;
      setBrakeMode();
    } else if (DriverStation.isDisabled() && brakeMode) {
      brakeMode = false;
      setCoastMode();
    }
    // io.updateInputs(inputs);
    // Logger.getInstance().processInputs("Indexer", inputs);
    /*if (true) {
    // double setpoint = positionSet.getDouble(0);
    double motorSpeedEntryDouble = motorSpeedEntry.getDouble(0);
    double p = kP.getDouble(Constants.INDEXER_ARM_P_VALUE);
    double i = kI.getDouble(Constants.INDEXER_ARM_I_VALUE);
    double d = kD.getDouble(Constants.INDEXER_ARM_D_VALUE);
    double iz = kIz.getDouble(0);
    double ff = kFF.getDouble(0);
    double minOut = kMinOutput.getDouble(Constants.INDEXER_ARM_PID_MIN_OUTPUT);
    double maxOut = kMaxOutput.getDouble(Constants.INDEXER_ARM_PID_MAX_OUTPUT);
    double maxVelocity = kMaxVelocity.getDouble(Constants.INDEXER_ARM_ROTATE_MAX_SPEED);
    double maxAccel = kMaxAccel.getDouble(Constants.INDEXER_ARM_ROTATE_MAX_ACCELERATION);

    if (preP != p) {
      pidController.setP(p);
      preP = p;
    }

    if (preI != i) {
      pidController.setI(i);
      preI = i;
    }

    if (preD != d) {
      pidController.setD(d);
      preD = d;
    }

    if (preIz != iz) {
      // pidController.setIZone(iz);
      preIz = iz;
    }

    if (preFF != ff) {
      // pidController.setFF(ff);
      preFF = ff;
    }

    if (preMinOutput != minOut || preMaxOutput != maxOut) {
      pidController.setOutputRange(minOut, maxOut);
      preMinOutput = minOut;
      preMaxOutput = maxOut;
    }

    if (preMaxVelocity != maxVelocity) {
      pidController.setSmartMotionMaxVelocity(maxVelocity, 0);
      preMaxVelocity = maxVelocity;
    }

    if (preMaxAccel != maxAccel) {
      pidController.setSmartMotionMaxAccel(maxAccel, 0);
      preMaxAccel = maxAccel;
    }*/

    // if (Math.abs(indexerRotationMotor.getEncoder().getPosition() - prevSetpoint) >= 2) {
    //   if (motorSpeed < 10e-4) {
    //     indexerRotationMotor.setVoltage(0);

    //   } else {
    //     indexerRotationMotor.setVoltage(
    //         motorSpeed
    //             * (indexerRotationMotor.getEncoder().getPosition() > prevSetpoint ? -1 : 1));
    //   }
    // }
    // if (Math.abs(motorSpeedEntryDouble - motorSpeed) >= 10e-7) {
    //  motorSpeed = motorSpeedEntryDouble; // motorSpeedEntry.getDouble(0);
    // }
    if (Math.abs(prevSetpoint - setpoint) >= 10e-7) {
      pidController.setReference(setpoint, ControlType.kPosition);
      prevSetpoint = setpoint;
    }
  }

  public void grabberIntake() {
    if (isOpen) {
      indexerGrabbingMotor.set(-0.25);
    } else {
      indexerGrabbingMotor.set(0.25);
    }
  }

  public void grabberOff() {
    indexerGrabbingMotor.stopMotor();
  }

  public void grabberEject() {
    if (isOpen) {
      indexerGrabbingMotor.set(0.25);
    } else {
      indexerGrabbingMotor.set(-0.25);
    }
  }

  public void rotateUp() {
    indexerRotationMotor.set(motorSpeed);
  }

  public void rotateDown() {
    indexerRotationMotor.set(-motorSpeed);
  }

  public void rotateOff() {
    indexerRotationMotor.stopMotor();
  }

  public void intake(PieceMode pieceMode) {
    if (pieceMode == PieceMode.CONE) {
      close();
    } else {
      open();
    }
    grabberIntake();
    setDown();
  }

  public void setCarrying() {
    grabberHoldPiece();
    setUp();
  }

  public void setReady() {
    grabberOff();
    setUp();
  }

  public void setHoldingLow() {
    grabberHoldPiece();
    setScoringPosition();
  }

  public void score() {
    setScoringPosition();
    grabberEject();
  }

  public void grabberHoldPiece() {
    if (isOpen) {
      indexerGrabbingMotor.set(-1 * Constants.INDEXER_HOLD_SPEED);
    } else {
      indexerGrabbingMotor.set(Constants.INDEXER_HOLD_SPEED);
    }
  }

  public void transferPiece() {
    if (isOpen) {
      indexerGrabbingMotor.set(Constants.INDEXER_TRANSFER_SPEED);
    } else {
      indexerGrabbingMotor.set(-Constants.INDEXER_TRANSFER_SPEED);
    }
  }

  public void setDown() {
    if (isOpen) {
      setpoint = Constants.INDEXER_CUBE_POSITION;
    } else {
      setpoint = Constants.INDEXER_CONE_POSITION;
    }
  }

  public void setUp() {
    setpoint = Constants.INDEXER_STARTING_POSITION;
  }

  public void setScoringPosition() {
    setpoint = Constants.INDEXER_SCORING_POSITION;
  }

  public void open() {
    indexerSolenoid.set(true);
    isOpen = true;
  }

  public void close() {
    indexerSolenoid.set(false);
    isOpen = false;
  }

  public void toggleOpenClose() {
    isOpen = !isOpen;
    indexerSolenoid.set(isOpen);
  }

  public double getArmRotation() {
    return indexerRotationMotor.getEncoder().getPosition();
  }

  public void pushAgainstHardstop() {
    indexerRotationMotor.set(Constants.INDEXER_ARM_ROTATE_STALL_SPEED);
    grabberHoldPiece();
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
    tab.addDouble("Current Rotation (amps)", () -> indexerRotationMotor.getOutputCurrent());
    tab.addDouble("Current Intake (amps)", () -> indexerGrabbingMotor.getOutputCurrent());
    if (false) {
      kP =
          tab.add("P", Constants.INDEXER_ARM_P_VALUE)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kI =
          tab.add("I", Constants.INDEXER_ARM_I_VALUE)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kD =
          tab.add("D", Constants.INDEXER_ARM_D_VALUE)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kIz = tab.add("Iz", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
      kFF = tab.add("FF", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

      kMaxOutput =
          tab.add("Max Output", Constants.INDEXER_ARM_PID_MAX_OUTPUT)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kMinOutput =
          tab.add("Min Output", Constants.INDEXER_ARM_PID_MIN_OUTPUT)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kMaxVelocity =
          tab.add("Max Velocity", Constants.INDEXER_ARM_ROTATE_MAX_SPEED)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kMaxAccel =
          tab.add("Max Accell", Constants.INDEXER_ARM_ROTATE_MAX_ACCELERATION)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();

      motorSpeedEntry =
          tab.add("Motor Speed", .015).withWidget(BuiltInWidgets.kTextView).getEntry();

      positionSet =
          tab.add("Set Position", 0)
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(0, 0)
              .getEntry();
    }
  }
}
