package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.RobotContainer.ScoringHeight;

public class ElevatorToScoringHeightCommand extends CommandBase {
  private RobotContainer robotContainer;
  private PieceMode prevPieceMode;
  private ScoringHeight prevScoringHeight;

  public ElevatorToScoringHeightCommand(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.indexerSubsystem);
    addRequirements(robotContainer.elevatorSubsystem);
    addRequirements(robotContainer.extenderSubsystem);
    addRequirements(robotContainer.holderSubsystem);
  }

  @Override
  public void initialize() {
    prevPieceMode = robotContainer.pieceMode;
    prevScoringHeight = robotContainer.scoringHeight;
    if (prevScoringHeight == ScoringHeight.MEDIUM) {
      robotContainer.elevatorSubsystem.goToMiddleScoringPosition(prevPieceMode);
    } else {
      robotContainer.elevatorSubsystem.goToHighScoringPosition(prevPieceMode);
    }
    robotContainer.extenderSubsystem.goToStartingPosition();
    robotContainer.holderSubsystem.close();
    robotContainer.indexerSubsystem.rotateOff();
    robotContainer.extenderSubsystem.disengageStablizer();
  }

  @Override
  public void execute() {
    // prevent changing piece mode
    if (prevPieceMode != robotContainer.pieceMode) {
      robotContainer.pieceMode = prevPieceMode;
    }

    // adjust to changing height
    if (prevScoringHeight != robotContainer.scoringHeight) {
      prevScoringHeight = robotContainer.scoringHeight;
      if (prevScoringHeight == ScoringHeight.MEDIUM) {
        robotContainer.elevatorSubsystem.goToMiddleScoringPosition(prevPieceMode);
      } else {
        robotContainer.elevatorSubsystem.goToHighScoringPosition(prevPieceMode);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return robotContainer.elevatorSubsystem.isHigherThanNeutral();
  }
}
