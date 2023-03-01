// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.PieceMode;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax elevatorBaseMotorOne, elevatorBaseMotorTwo;
  private RelativeEncoder relativeEncoder;
  private DigitalInput magLimitSwitch;
  private SparkMaxPIDController pidController;
  private GenericEntry positionSet;
  private GenericEntry kP,
      kI,
      kD,
      kIz,
      kFF,
      kMaxOutput,
      kMinOutput,
      kMaxVelocity,
      kMaxAccel,
      motorSpeedEntry;
  private double preP,
      preI,
      preD,
      preIz,
      preFF,
      preMaxOutput,
      preMinOutput,
      preMaxVelocity,
      preMaxAccel;
  private boolean brakeMode;
  private double prevSetpoint;
  private double setPoint;
  private double motorSpeed;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorBaseMotorOne =
        new CANSparkMax(Constants.ELEVATOR_BASE_MOTOR_ONE_CAN_ID, MotorType.kBrushless);
    elevatorBaseMotorTwo =
        new CANSparkMax(Constants.ELEVATOR_BASE_MOTOR_TWO_CAN_ID, MotorType.kBrushless);
    elevatorBaseMotorOne.restoreFactoryDefaults();
    elevatorBaseMotorTwo.restoreFactoryDefaults();
    // magLimitSwitch = new DigitalInput(Constants.ELEVATOR_MAGNETIC_LIMIT_SWITCH_CHANNEL);
    elevatorBaseMotorTwo.follow(elevatorBaseMotorOne, true);
    elevatorBaseMotorOne
        .getEncoder()
        .setPositionConversionFactor(Constants.ELEVATOR_INCHES_PER_ROTATION);
    elevatorBaseMotorOne.getEncoder().setPosition(Constants.ELEVATOR_STARTING_POSITION_INCHES);
    setBrakeMode();
    pidController = elevatorBaseMotorOne.getPIDController();
    pidController.setFeedbackDevice(elevatorBaseMotorOne.getEncoder());
    pidController.setReference(
        Constants.ELEVATOR_STARTING_POSITION_INCHES, ControlType.kSmartMotion);
    prevSetpoint = Constants.ELEVATOR_STARTING_POSITION_INCHES;
    setPoint = prevSetpoint;
    setupShuffleboard();
    motorSpeed = .4;
    // set PID coefficients
    pidController.setP(Constants.ELEVATOR_P_VALUE);
    pidController.setI(Constants.ELEVATOR_I_VALUE);
    pidController.setD(Constants.ELEVATOR_D_VALUE);
    pidController.setOutputRange(
        Constants.ELEVATOR_PID_MIN_OUTPUT, Constants.ELEVATOR_PID_MAX_OUTPUT);
    pidController.setIZone(0);
    pidController.setFF(0);
    pidController.setSmartMotionMaxVelocity(Constants.ELEVATOR_MAX_SPEED_RPM, 0);
    pidController.setSmartMotionMaxAccel(Constants.ELEVATOR_MAX_ACCELERATION_RPM2, 0);
    pidController.setSmartMotionAllowedClosedLoopError(0.05, 0);
    elevatorBaseMotorOne.burnFlash();
    elevatorBaseMotorTwo.burnFlash();
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
    double motorSpeedEntryDouble = motorSpeedEntry.getDouble(.2);
    if (motorSpeed != motorSpeedEntryDouble) {
      motorSpeed = motorSpeedEntryDouble;
    }
    /*if (DriverStation.isEnabled()) { // && Constants.TUNING_MODE) {
      double p = kP.getDouble(Constants.ELEVATOR_P_VALUE);
      double i = kI.getDouble(Constants.ELEVATOR_I_VALUE);
      double d = kD.getDouble(Constants.ELEVATOR_D_VALUE);
      double iz = kIz.getDouble(0);
      double ff = kFF.getDouble(0);
      double minOut = kMinOutput.getDouble(Constants.ELEVATOR_PID_MIN_OUTPUT);
      double maxOut = kMaxOutput.getDouble(Constants.ELEVATOR_PID_MAX_OUTPUT);
      double maxVelocity = kMaxVelocity.getDouble(Constants.ELEVATOR_MAX_SPEED_RPM);
      double maxAccel = kMaxAccel.getDouble(Constants.ELEVATOR_MAX_ACCELERATION_RPM2);
      double setpoint = positionSet.getDouble(0);
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
      }


    }*/
    if (Math.abs(prevSetpoint - setPoint) >= 10e-7) {
      pidController.setReference(setPoint, ControlType.kSmartMotion);
      prevSetpoint = setPoint;
    }
  }

  public void goToTransferPosition(PieceMode pieceMode) {
    if (pieceMode == PieceMode.CONE) {
      setPoint = Constants.ELEVATOR_CONE_TRANSFER_POSITION_INCHES;
    } else {
      setPoint = Constants.ELEVATOR_CUBE_TRANSFER_POSITION_INCHES;
    }
  }

  public void goToMiddleScoringPosition() {
    setPoint = Constants.ELEVATOR_MID_CONE_POSITION_INCHES;
  }

  public void goToHighScoringPosition() {
    setPoint = Constants.ELEVATOR_HIGH_CONE_POSITION_INCHES;
  }

  public void goToStartingPosition() {
    setPoint = Constants.ELEVATOR_STARTING_POSITION_INCHES;
  }

  public double limit(double value) {
    return value > .25 ? .25 : value < -.25 ? -.25 : value;
  }

  public void setBrakeMode() {
    elevatorBaseMotorOne.setIdleMode(IdleMode.kBrake);
    elevatorBaseMotorTwo.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    elevatorBaseMotorOne.setIdleMode(IdleMode.kCoast);
    elevatorBaseMotorTwo.setIdleMode(IdleMode.kCoast);
  }

  public void goUp() {
    elevatorBaseMotorOne.set(motorSpeed);
  }

  public void goDown() {
    elevatorBaseMotorOne.set(-motorSpeed);
  }

  public void off() {
    elevatorBaseMotorOne.stopMotor();
  }

  public boolean isAtSetpoint() {
    return Math.abs(elevatorBaseMotorOne.getEncoder().getPosition() - setPoint)
        < Constants.ELEVATOR_DEADBAND;
  }

  public void setToMidCone() {
    pidController.setReference(
        Constants.ELEVATOR_MID_CONE_POSITION_INCHES, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setToNeutralPosition() {
    pidController.setReference(
        Constants.ELEVATOR_NEUTRAL_POSITION_INCHES, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setToHighCone() {
    pidController.setReference(
        Constants.ELEVATOR_HIGH_CONE_POSITION_INCHES, CANSparkMax.ControlType.kSmartMotion);
  }

  public void reset() {
    elevatorBaseMotorOne.getEncoder().setPosition(0);
  }

  public Double getPosition() {
    return elevatorBaseMotorOne.getEncoder().getPosition();
  }

  public boolean getMagLimitSwitch() {
    return magLimitSwitch.get();
  }

  private void setupShuffleboard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("elevator");
    // tab.addBoolean("MagLimitSwitch", () -> in0.get());
    tab.addDouble("Pos", () -> elevatorBaseMotorOne.getEncoder().getPosition());
    tab.addDouble("Vel", () -> elevatorBaseMotorOne.getEncoder().getVelocity());
    tab.addDouble(
        "PosFactor", () -> elevatorBaseMotorOne.getEncoder().getPositionConversionFactor());
    tab.addDouble(
        "VelFactor", () -> elevatorBaseMotorOne.getEncoder().getVelocityConversionFactor());
    if (true) { // Constants.TUNING_MODE) {
      kP = tab.add("P", Constants.ELEVATOR_P_VALUE).withWidget(BuiltInWidgets.kTextView).getEntry();
      kI = tab.add("I", Constants.ELEVATOR_I_VALUE).withWidget(BuiltInWidgets.kTextView).getEntry();
      kD = tab.add("D", Constants.ELEVATOR_D_VALUE).withWidget(BuiltInWidgets.kTextView).getEntry();
      kIz = tab.add("Iz", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
      kFF = tab.add("FF", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

      kMaxOutput =
          tab.add("Max Output", Constants.ELEVATOR_PID_MAX_OUTPUT)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kMinOutput =
          tab.add("Min Output", Constants.ELEVATOR_PID_MIN_OUTPUT)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kMaxVelocity =
          tab.add("Max Velocity", Constants.ELEVATOR_MAX_SPEED_RPM)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kMaxAccel =
          tab.add("Max Accell", Constants.ELEVATOR_MAX_ACCELERATION_RPM2)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kMaxAccel =
          motorSpeedEntry =
              tab.add("Motor Speed", .4).withWidget(BuiltInWidgets.kTextView).getEntry();
      positionSet =
          tab.add("Set Position", 0)
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(0, 0)
              .getEntry();
    }
  }
}
