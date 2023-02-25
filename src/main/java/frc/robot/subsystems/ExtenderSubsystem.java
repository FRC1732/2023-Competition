// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtenderSubsystem extends SubsystemBase {
  private CANSparkMax extenderMotor;
  private DigitalInput extenderMagneticLimitSwitch;
  private SparkMaxPIDController pidController;

  private GenericEntry positionSet;

  private GenericEntry kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new IntakeSubsystem. */
  public ExtenderSubsystem() {
    extenderMotor = new CANSparkMax(Constants.EXTENDER_MOTOR_CAN_ID, MotorType.kBrushless);
    extenderMagneticLimitSwitch = new DigitalInput(Constants.EXTENDER_MAGNETIC_LIMIT_SWITCH);
    extenderMotor.restoreFactoryDefaults();
    pidController = extenderMotor.getPIDController();
    pidController.setReference(extenderMotor.getEncoder().getPosition(), ControlType.kPosition);

    setupShuffleboard();

    pidController.setP(kP.getDouble(1));
    pidController.setI(kI.getDouble(0));
    pidController.setD(kD.getDouble(0));
    pidController.setIZone(kIz.getDouble(0));
    pidController.setFF(kFF.getDouble(0));
    pidController.setOutputRange(kMinOutput.getDouble(-.25), kMinOutput.getDouble(.25));
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
    if (Constants.TUNING_MODE) {
      pidController.setP(kP.getDouble(1));
      pidController.setI(kI.getDouble(0));
      pidController.setD(kD.getDouble(0));
      pidController.setIZone(kIz.getDouble(0));
      pidController.setFF(kFF.getDouble(0));
      pidController.setOutputRange(kMinOutput.getDouble(-.25), kMaxOutput.getDouble(.25));
      pidController.setReference(positionSet.getDouble(0), ControlType.kPosition);
    }
  }

  private void setupShuffleboard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("Extender");
    // tab.addBoolean("MagLimitSwitch", () -> in0.get());
    tab.addDouble("Pos", () -> extenderMotor.getEncoder().getPosition());
    tab.addDouble("Vel", () -> extenderMotor.getEncoder().getVelocity());
    tab.addDouble("PosFactor", () -> extenderMotor.getEncoder().getPositionConversionFactor());
    tab.addDouble("VelFactor", () -> extenderMotor.getEncoder().getVelocityConversionFactor());
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
