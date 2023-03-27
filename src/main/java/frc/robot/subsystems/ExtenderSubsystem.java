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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.PieceMode;

@SuppressWarnings("unused")
public class ExtenderSubsystem extends SubsystemBase {
  private CANSparkMax extenderMotor;
  private DigitalInput extenderMagneticLimitSwitch;
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
  private double setPoint = 0;
  // private SuppliedValueWidget updatePID;
  private boolean brakeMode;
  private double prevSetpoint;
  /** Creates a new IntakeSubsystem. */
  public ExtenderSubsystem() {
    extenderMotor = new CANSparkMax(Constants.EXTENDER_MOTOR_CAN_ID, MotorType.kBrushless);
    extenderMagneticLimitSwitch = new DigitalInput(Constants.EXTENDER_MAGNETIC_LIMIT_SWITCH);
    extenderMotor.restoreFactoryDefaults();
    extenderMotor.setInverted(true);
    extenderMotor.getEncoder().setPositionConversionFactor(Constants.EXTENDER_INCHES_PER_ROTATION);
    extenderMotor.getEncoder().setPosition(0);
    setBrakeMode();
    pidController = extenderMotor.getPIDController();
    // pidController.setFeedbackDevice(extenderMotor.getEncoder());
    pidController.setReference(0, ControlType.kSmartMotion);
    prevSetpoint = 0;
    // setupShuffleboard();

    pidController.setP(Constants.EXTENDER_P_VALUE);
    pidController.setI(Constants.EXTENDER_I_VALUE);
    pidController.setD(Constants.EXTENDER_D_VALUE);
    pidController.setIZone(0);
    pidController.setFF(0);
    pidController.setSmartMotionMaxVelocity(Constants.EXTENDER_MAX_SPEED_RPM, 0);
    pidController.setSmartMotionMaxAccel(Constants.EXTENDER_MAX_ACCELERATION_RPM2, 0);
    pidController.setOutputRange(
        Constants.EXTENDER_PID_MIN_OUTPUT, Constants.EXTENDER_PID_MAX_OUTPUT);
    pidController.setSmartMotionAllowedClosedLoopError(0.1, 0);
    extenderMotor.burnFlash();
  }

  public void setCoastMode() {
    extenderMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBrakeMode() {
    extenderMotor.setIdleMode(IdleMode.kBrake);
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

  public boolean isAtSetpoint() {
    boolean temp =
        Math.abs(extenderMotor.getEncoder().getPosition() - setPoint) < Constants.EXTENDER_DEADBAND;
    if (temp) {
      System.out.println("Extender at setpoint");
    }
    return temp;
  }

  @Override
  public void periodic() {
    // if (DriverStation.isEnabled() && !brakeMode) {
    //   brakeMode = true;
    //   setBrakeMode();
    // } else if (DriverStation.isDisabled() && brakeMode) {
    //   brakeMode = false;
    //   setCoastMode();
    // }
    /*if (DriverStation.isEnabled()) { // } && Constants.TUNING_MODE) {
      double p = kP.getDouble(Constants.EXTENDER_P_VALUE);
      double i = kI.getDouble(Constants.EXTENDER_I_VALUE);
      double d = kD.getDouble(Constants.EXTENDER_D_VALUE);
      double iz = kIz.getDouble(0);
      double ff = kFF.getDouble(0);
      double minOut = kMinOutput.getDouble(Constants.EXTENDER_PID_MIN_OUTPUT);
      double maxOut = kMaxOutput.getDouble(Constants.EXTENDER_PID_MAX_OUTPUT);
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

      if (Math.abs(prevSetpoint - setpoint) >= 10e-7) {
        pidController.setReference(setpoint, ControlType.kSmartMotion);
        prevSetpoint = setpoint;
      }
    }*/
    if (Math.abs(prevSetpoint - setPoint) >= 10e-7) {
      pidController.setReference(setPoint, ControlType.kSmartMotion);
      prevSetpoint = setPoint;
    }
  }

  public void goToMiddleScoringPosition(PieceMode pieceMode) {
    if (pieceMode == PieceMode.CONE) {
      setPoint = Constants.EXTENDER_MID_CONE_POSITION_INCHES;
    } else {
      setPoint = Constants.EXTENDER_MID_CONE_POSITION_INCHES - 3;
    }
  }

  public void goToHighScoringPosition(PieceMode pieceMode) {
    if (pieceMode == PieceMode.CONE) {
      setPoint = Constants.EXTENDER_HIGH_CONE_POSITION_INCHES;
    } else {
      setPoint = Constants.EXTENDER_HIGH_CONE_POSITION_INCHES - 2.5;
    }
  }

  public void goToStartingPosition() {
    setPoint = Constants.EXTENDER_STARTING_POSITION_INCHES;
  }

  private void setupShuffleboard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("Extender");
    // tab.addBoolean("MagLimitSwitch", () -> in0.get());
    tab.addDouble("Pos", () -> extenderMotor.getEncoder().getPosition());
    tab.addDouble("Vel", () -> extenderMotor.getEncoder().getVelocity());
    tab.addDouble("PosFactor", () -> extenderMotor.getEncoder().getPositionConversionFactor());
    tab.addDouble("VelFactor", () -> extenderMotor.getEncoder().getVelocityConversionFactor());
    if (true) { // Constants.TUNING_MODE) {
      kP = tab.add("P", Constants.EXTENDER_P_VALUE).withWidget(BuiltInWidgets.kTextView).getEntry();
      kI = tab.add("I", Constants.EXTENDER_I_VALUE).withWidget(BuiltInWidgets.kTextView).getEntry();
      kD = tab.add("D", Constants.EXTENDER_D_VALUE).withWidget(BuiltInWidgets.kTextView).getEntry();
      kIz = tab.add("Iz", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
      kFF = tab.add("FF", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
      // updatePID = tab.addBoolean("Update PID",
      // ()->updatePIDbool).withWidget(BuiltInWidgets.kToggleButton);

      // updatePID = tab.addBoolean("Update PID",
      // ()->updatePIDbool).withWidget(BuiltInWidgets.kToggleButton);

      kMaxOutput =
          tab.add("Max Output", Constants.EXTENDER_PID_MAX_OUTPUT)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kMinOutput =
          tab.add("Min Output", Constants.EXTENDER_PID_MIN_OUTPUT)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kMaxVelocity =
          tab.add("Max Velocity", Constants.EXTENDER_MAX_SPEED_RPM)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kMaxAccel =
          tab.add("Max Accell", Constants.EXTENDER_MAX_ACCELERATION_RPM2)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      positionSet =
          tab.add("Set Position", 0)
              .withWidget(BuiltInWidgets.kTextView)
              .withPosition(0, 0)
              .getEntry();
    }
  }
}
