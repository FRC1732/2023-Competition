package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveDistance extends CommandBase {
  public enum Direction {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT;
  }

  private final Drivetrain drivetrain;
  private Translation2d startingPose;
  private double targetDistance;
  private Direction direction;
  private double speed;

  public DriveDistance(Drivetrain drivetrain, double distanceMeters) {
    this.drivetrain = drivetrain;
    targetDistance = distanceMeters;
    direction = Direction.FORWARD;
    speed = 0.2;
    addRequirements(drivetrain);
  }

  public DriveDistance(Drivetrain drivetrain, Direction direction, double distanceMeters) {
    this.drivetrain = drivetrain;
    targetDistance = distanceMeters;
    this.direction = direction;
    speed = 0.2;
    addRequirements(drivetrain);
  }

  public DriveDistance(Drivetrain drivetrain, double distanceMeters, double speed) {
    this.drivetrain = drivetrain;
    targetDistance = distanceMeters;
    direction = Direction.FORWARD;
    this.speed = speed;
    addRequirements(drivetrain);
  }

  public DriveDistance(
      Drivetrain drivetrain, Direction direction, double distanceMeters, double speed) {
    this.drivetrain = drivetrain;
    targetDistance = distanceMeters;
    this.direction = direction;
    this.speed = speed;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    drivetrain.enableFieldRelative();
    startingPose = drivetrain.getPose().getTranslation();
  }

  @Override
  public void execute() {
    double x, y;
    x = y = 0;

    switch (direction) {
      case BACKWARD:
        x = speed;
        break;
      case FORWARD:
        x = -1.0 * speed;
        break;
      case LEFT:
        y = speed;
        break;
      case RIGHT:
        y = -1.0 * speed;
        break;
      default:
        break;
    }

    drivetrain.drivePercentage(x, y, 0);
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
