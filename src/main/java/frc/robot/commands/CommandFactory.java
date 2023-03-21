package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.TransitionCommands.*;

public final class CommandFactory {
  private static final double FIELD_WIDTH_METERS = 8.02;

  private CommandFactory() {
    throw new IllegalStateException("attempted to instantiate static class");
  }

  public static Command getScoreWithHolderCommand(RobotContainer robotContainer) {
    return Commands.sequence(
        new ElevatorToScoringHeightCommand(robotContainer),
        new DeployExtenderCommand(robotContainer),
        new OpenHolderCommand(robotContainer),
        new RetractExtenderCommand(robotContainer),
        new ResetToReadyCommand(robotContainer)
            .until(() -> robotContainer.elevatorSubsystem.isAtSetpoint()));
  }

  public static Pose2d getAllianceCorrectedPose(Pose2d pose) {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      return pose;
    }
    return new Pose2d(pose.getX(), FIELD_WIDTH_METERS - pose.getY(), pose.getRotation().times(-1));
  }
}
