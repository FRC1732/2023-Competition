// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoDriving;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.CommandFactory;
import java.util.ArrayList;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a swerve drive.
 *
 * <p>This command outputs the raw desired Swerve Module States ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("MemberName")
public class HilltopSwerveControllerCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;
  private final TrajectoryConfig m_trajectoryConfig;
  private Pose2d m_endPose;
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;

  @SuppressWarnings("ParameterName")
  public HilltopSwerveControllerCommand(
      TrajectoryConfig trajectoryConfig,
      Pose2d endPose,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    m_trajectoryConfig = trajectoryConfig;
    m_endPose = endPose;
    m_pose = pose;
    m_kinematics = kinematics;

    m_controller = new HolonomicDriveController(xController, yController, thetaController);

    m_outputModuleStates = outputModuleStates;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_endPose = CommandFactory.getAllianceCorrectedPose(m_endPose);
    m_trajectory =
        getTrajectory(
            m_pose.get().getTranslation(), m_endPose.getTranslation(), m_trajectoryConfig);
    m_timer.reset();
    m_timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);

    var targetChassisSpeeds =
        m_controller.calculate(m_pose.get(), desiredState, m_endPose.getRotation());
    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }

  private static Trajectory getTrajectory(
      Translation2d start, Translation2d end, TrajectoryConfig config) {
    var angle = getTrajectoryAngle(start, end);
    return TrajectoryGenerator.generateTrajectory(
        new Pose2d(start.getX(), start.getY(), angle),
        new ArrayList<>(),
        new Pose2d(end.getX(), end.getY(), angle),
        config);
  }

  private static Rotation2d getTrajectoryAngle(Translation2d start, Translation2d end) {
    double xdist = end.getX() - start.getX();
    double ydist = end.getY() - start.getY();
    double angle = Math.atan2(ydist, xdist);
    return new Rotation2d(angle);
  }
}
