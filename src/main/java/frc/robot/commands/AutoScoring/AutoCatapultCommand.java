package frc.robot.commands.AutoScoring;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class AutoCatapultCommand extends InstantCommand {
  private RobotContainer robotContainer;
  double setpoint;

  public AutoCatapultCommand(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.indexerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void end(boolean interrupted) {
    robotContainer.indexerSubsystem.setUp();
  }

  @Override
  public boolean isFinished() {
    return robotContainer.indexerSubsystem.isAtSetpoint();
  }
}
