package frc.robot.commands;

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
  private double startingDistance;

  /**
   * Create a new TeleopSwerve command object.
   *
   * @param drivetrain the drivetrain subsystem instructed by this command
   */
  public DriveDistance(Drivetrain drivetrain, double distanceMeters) {
    this.drivetrain = drivetrain;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.enableFieldRelative();
    double tempx = drivetrain.getPose().getX();
    System.out.println("Starting X coord: " + tempx);
    startingDistance = tempx;
  }

  @Override
  public void execute() {
    drivetrain.drivePercentage(0.25, 0, 0);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drivePercentage(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    double tempx = drivetrain.getPose().getX();
    System.out.println("Current X coord: " + tempx);
    return tempx > startingDistance;
  }
}
