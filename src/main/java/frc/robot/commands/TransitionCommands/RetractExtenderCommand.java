package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.RobotContainer.ScoringHeight;

public class RetractExtenderCommand extends WaitCommand {
  private RobotContainer robotContainer;
  private PieceMode prevPieceMode;
  private ScoringHeight prevScoringHeight;

  public RetractExtenderCommand(RobotContainer robotContainer) {
    super(0.2);
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.elevatorSubsystem);
    addRequirements(robotContainer.extenderSubsystem);
    addRequirements(robotContainer.holderSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    prevPieceMode = robotContainer.pieceMode;
    prevScoringHeight = robotContainer.scoringHeight;
    robotContainer.holderSubsystem.open();
    if (prevScoringHeight == ScoringHeight.MEDIUM) {
      robotContainer.elevatorSubsystem.goToMiddleScoringPosition(prevPieceMode);
    } else {
      robotContainer.elevatorSubsystem.goToHighScoringPosition(prevPieceMode);
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

  // @Override
  // public boolean isFinished() {
  //   return robotContainer.extenderSubsystem.isAtSetpoint();
  // }
}
