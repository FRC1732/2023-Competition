package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.RobotContainer.RobotRotationMode;
import frc.robot.RobotContainer.ScoringHeight;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.FinishScorePressed;

public class StageGamePieceCommand extends CommandBase {
  private RobotContainer robotContainer;
  private RobotStateMachine robotStateMachine;
  private PieceMode prevPieceMode;
  private ScoringHeight prevScoringHeight;

  public StageGamePieceCommand(RobotContainer robotContainer, RobotStateMachine robotStateMachine) {
    this.robotContainer = robotContainer;
    this.robotStateMachine = robotStateMachine;
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
      robotContainer.elevatorSubsystem.goToMiddleScoringPosition();
    } else {
      robotContainer.elevatorSubsystem.goToHighScoringPosition();
    }
    robotContainer.extenderSubsystem.goToStartingPosition();
    robotContainer.holderSubsystem.close();
    robotContainer.indexerSubsystem.transferPiece();
    robotContainer.robotRotationMode = RobotRotationMode.DRIVER;
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
        robotContainer.elevatorSubsystem.goToMiddleScoringPosition();
      } else {
        robotContainer.elevatorSubsystem.goToHighScoringPosition();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    robotStateMachine.fireEvent(new FinishScorePressed());
  }

  @Override
  public boolean isFinished() {
    return robotContainer.elevatorSubsystem.isHigherThanNeutral();
  }
}
