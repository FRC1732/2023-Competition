package frc.robot.commands.AutoScoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoConeDropOffCommand extends CommandBase {
  private RobotContainer robotContainer;
  double setpoint;

  public AutoConeDropOffCommand(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.indexerSubsystem);
  }

  @Override
  public void initialize() {
    robotContainer.indexerSubsystem.setAutoPosition(Constants.INDEXER_CONE_POSITION);
    robotContainer.indexerSubsystem.dropOffPiece();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    robotContainer.indexerSubsystem.grabberOff();
    robotContainer.indexerSubsystem.rotateUp();
  }

  @Override
  public boolean isFinished() {
    return robotContainer.indexerSubsystem.isAtSetpoint();
  }
}
