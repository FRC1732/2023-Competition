package frc.robot.commands.AutoDriving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A base class that enables a swerve drive subsystem to use the hilltoplib auto library. */
public abstract class AutoSwerveDriveSubsystem extends SubsystemBase {
  /**
   * Gets the current position of the robot on the field.
   *
   * @return the position of the robot on the field.
   */
  public abstract Pose2d getPose();

  /**
   * Gets the swerve drive kinematics.
   *
   * @return the swerve drive kinematics.
   */
  public abstract SwerveDriveKinematics getKinematics();

  /**
   * Sets the states that the swerve modules will be updated to next.
   *
   * @param desiredStates the array of states to set the swerve modules to
   */
  public abstract void setModuleStates(SwerveModuleState[] desiredStates);
}
