// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.gyro.GyroIoADIS16470;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/*
 * WARNING: This assumes auto mode where the robot is facing the scoring goals. Other facing configurations will not work with this code implementation.
 */
public class AutoBalance extends CommandBase {
  private GyroIoADIS16470 imu;
  private Drivetrain drivetrain;
  private int count;

  private GenericEntry kP, kI, kD;
  private double preP, preI, preD;

  private PIDController pidController;

  public AutoBalance(GyroIoADIS16470 adis16470Gyro, Drivetrain drivetrain) {
    imu = adis16470Gyro;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    pidController =
        new PIDController(
            Constants.AUTOBALANCE_P_VALUE,
            Constants.AUTOBALANCE_I_VALUE,
            Constants.AUTOBALANCE_D_VALUE);
    pidController.enableContinuousInput(-1 * Math.PI, Math.PI);

    setupShuffleboard();
  }

  @Override
  public void initialize() {
    count = 0;
    System.out.println("Begin Balance");
  }

  @Override
  public void execute() {
    if (Constants.TUNING_MODE) {
      double p = kP.getDouble(Constants.AUTOBALANCE_P_VALUE);
      double i = kI.getDouble(Constants.AUTOBALANCE_I_VALUE);
      double d = kD.getDouble(Constants.AUTOBALANCE_D_VALUE);

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
    }

    double x =
        pidController.calculate(
                Math.toRadians(imu.xComplementary()), Constants.AUTOBALANCE_SET_POINT_RADIANS)
            / Math.PI
            * 0.2;

    drivetrain.drivePercentage(x, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    System.out.println("Balance Ended - Interrupted: " + interrupted);
  }

  @Override
  public boolean isFinished() {
    // flat is -180/180. Lets try to make this easier on ourselves
    double adjustedX = (360.0 + imu.xComplementary()) % 360.0;
    if (Math.toDegrees(Constants.AUTOBALANCE_SET_POINT_RADIANS)
                + Constants.AUTOBALANCE_LEVEL_TOLERANCE_DEGRESS
            > adjustedX
        && adjustedX
            > Math.toDegrees(Constants.AUTOBALANCE_SET_POINT_RADIANS)
                - Constants.AUTOBALANCE_LEVEL_TOLERANCE_DEGRESS) {
      count++;
      return count > 10;
    }

    count = 0;
    return false;
  }

  private void setupShuffleboard() {
    if (Constants.TUNING_MODE) {
      ShuffleboardTab tab;
      tab = Shuffleboard.getTab("autoBalance");

      kP =
          tab.add("P", Constants.AUTOBALANCE_P_VALUE)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kI =
          tab.add("I", Constants.AUTOBALANCE_I_VALUE)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kD =
          tab.add("D", Constants.AUTOBALANCE_D_VALUE)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();

      tab.addDouble("X Angle", () -> imu.xComplementary());
      tab.addDouble("Set Point", () -> Math.toDegrees(Constants.AUTOBALANCE_SET_POINT_RADIANS));
    }
  }
}
