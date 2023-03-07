package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.RobotContainer.ScoringHeight;

public class RetractExtenderCommand extends CommandBase {
  private RobotContainer robotContainer;
  private PieceMode prevPieceMode;
  private ScoringHeight prevScoringHeight;

  public RetractExtenderCommand(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.elevatorSubsystem);
    addRequirements(robotContainer.extenderSubsystem);
    addRequirements(robotContainer.holderSubsystem);
  }

  @Override
  public void initialize() {
    prevPieceMode = robotContainer.pieceMode;
    prevScoringHeight = robotContainer.scoringHeight;
    robotContainer.holderSubsystem.open();
    if (prevScoringHeight == ScoringHeight.MEDIUM) {
      robotContainer.elevatorSubsystem.goToMiddleScoringPosition();
    } else {
      robotContainer.elevatorSubsystem.goToHighScoringPosition();
    }
    robotContainer.extenderSubsystem.goToStartingPosition();
  }

  @Override
  public void execute() {
    // prevent changing piece mode
    if (prevPieceMode != robotContainer.pieceMode) {
      robotContainer.pieceMode = prevPieceMode;
    }
    // prevent changing scoring height
    if (prevScoringHeight != robotContainer.scoringHeight) {
      robotContainer.scoringHeight = prevScoringHeight;
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return robotContainer.extenderSubsystem.isAtSetpoint();
  }
}
