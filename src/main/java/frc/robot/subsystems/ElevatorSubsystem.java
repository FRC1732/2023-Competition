// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private CANSparkMax elevatorBaseMotorOne, elevatorBaseMotorTwo;
  private RelativeEncoder relativeEncoder;
  private DigitalInput magLimitSwitch;
  private SparkMaxPIDController pidController;
  private GenericEntry positionSet;
  private GenericEntry kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private boolean brakeMode;
  private double prevSetpoint;

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
    relativeEncoder =
        elevatorBaseMotorOne.getAlternateEncoder(
            Type.kQuadrature, Constants.ELEVATOR_TICKS_PER_ROTATION);
    relativeEncoder.setPositionConversionFactor(Constants.ELEVATOR_INCHES_PER_ROTATION);
    relativeEncoder.setMeasurementPeriod(Constants.ELEVATOR_MEAUSREMENT_PERIOD_MS);
    // relativeEncoder.setPosition(Constants.ELEVATOR_STARTING_POSITION_INCHES);
    pidController = elevatorBaseMotorOne.getPIDController();
    prevSetpoint = 0;
    pidController.setFeedbackDevice(relativeEncoder);

    // PID coefficients

    // kMaxOutput = 1;
    // kMinOutput = -1;

    setupShuffleboard();

    // set PID coefficients
    pidController.setP(Constants.ELEVATOR_P_VALUE);
    pidController.setI(Constants.ELEVATOR_I_VALUE);
    pidController.setD(Constants.ELEVATOR_D_VALUE);
    pidController.setIZone(0);
    pidController.setFF(0);
    pidController.setSmartMotionMaxVelocity(Constants.ELEVATOR_MAX_SPEED_RPM, 0);
    pidController.setSmartMotionMaxAccel(Constants.ELEVATOR_MAX_ACCELERATION_RPM2, 0);
    // pidController.setOutputRange(-.25, .25);
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
    if (Constants.TUNING_MODE) {
      // pidController.setP(kP.getDouble(Constants.ELEVATOR_P_VALUE));
      // pidController.setI(kI.getDouble(Constants.ELEVATOR_I_VALUE));
      // pidController.setD(kD.getDouble(Constants.ELEVATOR_D_VALUE));
      // pidController.setIZone(kIz.getDouble(0));
      // pidController.setFF(kFF.getDouble(0));
      // pidController.setOutputRange(kMinOutput.getDouble(-.25), kMaxOutput.getDouble(.25));
      if (Math.abs(prevSetpoint - positionSet.getDouble(0)) > 10e-7) {
        pidController.setReference(positionSet.getDouble(0), CANSparkMax.ControlType.kSmartMotion);
        prevSetpoint = positionSet.getDouble(0);
      }
    }
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
    elevatorBaseMotorOne.set(.1);
    elevatorBaseMotorTwo.set(-.1);
  }

  public void goDown() {
    elevatorBaseMotorOne.set(-.1);
    elevatorBaseMotorTwo.set(.1);
  }

  public void off() {
    elevatorBaseMotorOne.set(0);
  }

  public void setToMidCone() {
    pidController.setReference(
        Constants.ELEVATOR_MID_CONE_POSITION_INCHES, CANSparkMax.ControlType.kSmartMotion);
  }

  public void setToHighCone() {
    pidController.setReference(
        Constants.ELEVATOR_HIGH_CONE_POSITION_INCHES, CANSparkMax.ControlType.kSmartMotion);
  }

  public void reset() {
    relativeEncoder.setPosition(0);
  }

  public Double getPosition() {
    return relativeEncoder.getPosition();
  }

  public boolean getMagLimitSwitch() {
    return magLimitSwitch.get();
  }

  private void setupShuffleboard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("elevator");
    // tab.addBoolean("MagLimitSwitch", () -> in0.get());
    tab.addDouble("Pos", () -> relativeEncoder.getPosition());
    tab.addDouble("Vel", () -> relativeEncoder.getVelocity());
    tab.addDouble("PosFactor", () -> relativeEncoder.getPositionConversionFactor());
    tab.addDouble("VelFactor", () -> relativeEncoder.getVelocityConversionFactor());
    if (Constants.TUNING_MODE) {
      kP = tab.add("P", Constants.ELEVATOR_P_VALUE).withWidget(BuiltInWidgets.kTextView).getEntry();
      kI = tab.add("I", Constants.ELEVATOR_I_VALUE).withWidget(BuiltInWidgets.kTextView).getEntry();
      kD = tab.add("D", Constants.ELEVATOR_D_VALUE).withWidget(BuiltInWidgets.kTextView).getEntry();
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
