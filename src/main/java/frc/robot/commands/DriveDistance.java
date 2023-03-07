package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive based on the specified
 * values from the controller(s). This command is designed to be the default command for the
 * drivetrain subsystem.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: never
 *
 * <p>At End: stops the drivetrain
 */
public class DriveDistance extends CommandBase {

  private final Drivetrain drivetrain;
  private Translation2d startingPose;
  private double targetDistance;

  /**
   * Create a new TeleopSwerve command object.
   *
   * @param drivetrain the drivetrain subsystem instructed by this command
   */
  public DriveDistance(Drivetrain drivetrain, double distanceMeters) {
    this.drivetrain = drivetrain;
    targetDistance = distanceMeters;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.enableFieldRelative();
    startingPose = drivetrain.getPose().getTranslation();
  }

  @Override
  public void execute() {
    drivetrain.drivePercentage(0.20, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drivePercentage(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    double curDistance = startingPose.getDistance(drivetrain.getPose().getTranslation());
    return curDistance >= targetDistance;
  }
}
