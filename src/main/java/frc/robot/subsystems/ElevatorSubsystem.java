// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private GenericEntry velocitySet;
  private GenericEntry kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double Integral, Derivative;
  private double prevError;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorBaseMotorOne = new CANSparkMax(Constants.ELEVATOR_BASE_MOTOR_ONE_CAN_ID, MotorType.kBrushless);
    elevatorBaseMotorTwo = new CANSparkMax(Constants.ELEVATOR_BASE_MOTOR_TWO_CAN_ID, MotorType.kBrushless);
    elevatorBaseMotorOne.restoreFactoryDefaults();
    elevatorBaseMotorTwo.restoreFactoryDefaults();
    magLimitSwitch = new DigitalInput(Constants.ELEVATOR_MAGNETIC_LIMIT_SWITCH_CHANNEL);
    // motor2.setInverted(true);
    elevatorBaseMotorTwo.follow(elevatorBaseMotorOne, true);
    relativeEncoder = elevatorBaseMotorOne.getAlternateEncoder(Type.kQuadrature, 8192);
    relativeEncoder.setVelocityConversionFactor(1.0/300);
    pidController = elevatorBaseMotorOne.getPIDController();

    pidController.setFeedbackDevice(relativeEncoder);
    Integral = 0;
    Derivative = 0;
    prevError = 0;

    // PID coefficients

    // kMaxOutput = 1;
    // kMinOutput = -1;

    setupShuffleboard();

    // set PID coefficients
    pidController.setP(kP.getDouble(1));
    pidController.setI(kI.getDouble(0));
    pidController.setD(kD.getDouble(0));
    pidController.setIZone(kIz.getDouble(0));
    pidController.setFF(kFF.getDouble(0));
    pidController.setOutputRange(kMinOutput.getDouble(-.25), kMinOutput.getDouble(.25));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double r = velocitySet.getDouble(0);
    double p = kP.getDouble(1);
    double i = kI.getDouble(0);
    double d = kD.getDouble(0);
    double error = r - relativeEncoder.getVelocity();
    //if(error < 10e-4){
    //  Integral = 0;
    //}
    Integral+=error;
    Derivative = error - prevError;
    pidController.setP(kP.getDouble(1));
    pidController.setI(kI.getDouble(0));
    pidController.setD(kD.getDouble(0));
    pidController.setIZone(kIz.getDouble(0));
    pidController.setFF(kFF.getDouble(0));
    pidController.setOutputRange(kMinOutput.getDouble(-.25), kMaxOutput.getDouble(.25));
  //  if (r != rotationSetPoint) {
   //   rotationSetPoint = r;
      //pidController.setReference(r, CANSparkMax.ControlType.kVelocity);
  elevatorBaseMotorOne.set(p *error + i * Integral + d * Derivative);
  }

  public double limit(double value) {

    return value > .25 ? .25 : value < -.25 ? -.25 : value;
  }

  public void on() {
    elevatorBaseMotorOne.set(.1);
  }

  public void off() {
    elevatorBaseMotorOne.set(0);
  }

  public void reset() {
    relativeEncoder.setPosition(0);
  }
  public double getPosition() {
    return relativeEncoder.getPosition();
  }
  public double 

  private void setupShuffleboard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("elevator");
    // tab.addBoolean("MagLimitSwitch", () -> in0.get());
    tab.addDouble("Pos", () -> relativeEncoder.getPosition());
    tab.addDouble("Vel", () -> relativeEncoder.getVelocity());
    tab.addDouble("PosFactor", () -> relativeEncoder.getPositionConversionFactor());
    tab.addDouble("VelFactor", () -> relativeEncoder.getVelocityConversionFactor());
    kP = tab.add("P", .9).withWidget(BuiltInWidgets.kTextView).getEntry();
    kI = tab.add("I", .1).withWidget(BuiltInWidgets.kTextView).getEntry();
    kD = tab.add("D", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    kIz = tab.add("Iz", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    kFF = tab.add("FF", 0).withWidget(BuiltInWidgets.kTextView).getEntry();

    kMinOutput = tab.add("Max Output", .25).withWidget(BuiltInWidgets.kTextView).getEntry();
    kMaxOutput = tab.add("Min Output", -.25).withWidget(BuiltInWidgets.kTextView).getEntry();
    velocitySet = tab.add("Set Velocity", 0).withWidget(BuiltInWidgets.kTextView).withPosition(0, 0)
        .getEntry();

    // tab.add("PIDController",pidController).withWidget(BuiltInWidgets.kPIDController).getEntry();
  }

}