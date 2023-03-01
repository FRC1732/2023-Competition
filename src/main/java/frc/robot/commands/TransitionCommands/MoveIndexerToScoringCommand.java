package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;

public class MoveIndexerToScoringCommand extends CommandBase {
  private RobotContainer robotContainer;
  private PieceMode prevPieceMode;

  public MoveIndexerToScoringCommand(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.indexerSubsystem);
    addRequirements(robotContainer.elevatorSubsystem);
    addRequirements(robotContainer.extenderSubsystem);
    addRequirements(robotContainer.holderSubsystem);
  }

  @Override
  public void initialize() {
    prevPieceMode = robotContainer.pieceMode;
    robotContainer.elevatorSubsystem.setToNeutralPosition();
    robotContainer.extenderSubsystem.goToStartingPosition();
    robotContainer.holderSubsystem.open();
    robotContainer.indexerSubsystem.setCarrying();
    if (prevPieceMode == PieceMode.CONE) {
      robotContainer.indexerSubsystem.close();
    } else {
      robotContainer.indexerSubsystem.open();
    }
  }

  @Override
  public void execute() {
    // prevent changing piece mode
    if (prevPieceMode != robotContainer.pieceMode) {
      robotContainer.pieceMode = prevPieceMode;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return robotContainer.indexerSubsystem.isAtSetpoint();
  }
}
