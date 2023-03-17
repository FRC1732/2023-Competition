package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.TransitionCommands.*;

public final class CommandFactory {
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
}
