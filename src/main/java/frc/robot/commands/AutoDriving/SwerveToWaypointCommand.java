// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoDriving;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

/** Add your docs here. */
public class SwerveToWaypointCommand extends HilltopSwerveControllerCommand {

  /** Creates a new Auto10Feet. */
  public SwerveToWaypointCommand(
      AutoSwerveDriveSubsystem drivetrain,
      Pose2d waypoint,
      boolean stopAtEnd,
      double maxAngularVelocity,
      double maxAngularAcceleration,
      double maxVelocity,
      double maxAcceleration) {
    super(
        getDefaultTrajectoryConfig(drivetrain, stopAtEnd, maxVelocity, maxAcceleration),
        waypoint,
        drivetrain::getPose, // Functional interface to feed supplier
        drivetrain.getKinematics(),
        // Position controllers
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        getThetaController(maxAngularVelocity, maxAngularAcceleration),
        drivetrain::setModuleStates,
        drivetrain);
  }

  public SwerveToWaypointCommand(
      AutoSwerveDriveSubsystem drivetrain, Pose2d waypoint, boolean stopAtEnd) {
    super(
        getDefaultTrajectoryConfig(
            drivetrain,
            stopAtEnd,
            DrivetrainConstants.AUTO_MAX_SPEED_METERS_PER_SECOND,
            DrivetrainConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
        waypoint,
        drivetrain::getPose, // Functional interface to feed supplier
        drivetrain.getKinematics(),
        // Position controllers
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        getThetaController(
            DrivetrainConstants.AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
            DrivetrainConstants.AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED),
        drivetrain::setModuleStates,
        drivetrain);
  }

  private static ProfiledPIDController getThetaController(
      double maxAngularVelocity, double maxAngularAcceleration) {
    var profileConstraints =
        new TrapezoidProfile.Constraints(
            maxAngularVelocity, // MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            maxAngularAcceleration); // MAX_ANGULAR_ACCELERATION * Math.PI / 180 * 5);
    var thetaController = new ProfiledPIDController(7, 0, 0, profileConstraints);
    thetaController.enableContinuousInput(Math.PI * -1, Math.PI);
    return thetaController;
  }

  private static TrajectoryConfig getDefaultTrajectoryConfig(
      AutoSwerveDriveSubsystem drivetrain,
      boolean stopAtEnd,
      double maxVelocity,
      double maxAcceleration) {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
            maxVelocity, // MAX_VELOCITY_METERS_PER_SECOND/3,
            maxAcceleration); // MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    // Add kinematics to ensure max speed is actually obeyed
    config.setKinematics(drivetrain.getKinematics());
    if (stopAtEnd) config.setEndVelocity(0.0);
    return config;
  }
}
