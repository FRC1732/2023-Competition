// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.gyro.GyroIoADIS16470;
import frc.robot.subsystems.drivetrain.Drivetrain;

/*
 * WARNING: This assumes auto mode where the robot is facing the scoring goals. Other facing configurations will not work with this code implementation.
 */
public class AutoBalance extends CommandBase {
  private GyroIoADIS16470 imu;
  private Drivetrain drivetrain;
  private int count;

  private final double SET_POINT_RADIANS_COMPETITION = Math.PI;
  private final double SET_POINT_RADIANS_PROTOBOT = 0;
  private final double SET_POINT_RADIANS = SET_POINT_RADIANS_COMPETITION;

  private final double LEVEL_TOLERANCE_DEGRESS = 3.0;

  private PIDController pidController;

  public AutoBalance(GyroIoADIS16470 adis16470Gyro, Drivetrain drivetrain) {
    imu = adis16470Gyro;
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    pidController =
        new PIDController(
            10, 0, 1); // I have a feeling D is important here but no idea what it should be
    pidController.enableContinuousInput(-1 * Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    System.out.println("Begin Balance");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x =
        pidController.calculate(Math.toRadians(imu.xComplementary()), SET_POINT_RADIANS)
            / Math.PI
            * 0.2;

    drivetrain.drivePercentage(x, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    System.out.println("Balance Ended - Interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // flat is -180/180. Lets try to make this easier on ourselves
    double adjustedX = (360.0 + imu.xComplementary()) % 360.0;
    if (Math.toDegrees(SET_POINT_RADIANS) + LEVEL_TOLERANCE_DEGRESS > adjustedX
        && adjustedX > Math.toDegrees(SET_POINT_RADIANS) - LEVEL_TOLERANCE_DEGRESS) {
      count++;
      return count > 10;
    }

    count = 0;
    return false;
  }
}
