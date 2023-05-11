package frc.robot.commands.AutoScoring;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class AutoIndexerPositionCommand extends InstantCommand {
  private RobotContainer robotContainer;
  double setpoint;

  public AutoIndexerPositionCommand(RobotContainer robotContainer, double setpoint) {
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.indexerSubsystem);
    this.setpoint = setpoint;
  }

  @Override
  public void initialize() {
    robotContainer.indexerSubsystem.setAutoPosition(setpoint);
  }

  @Override
  public void execute() {}
}
