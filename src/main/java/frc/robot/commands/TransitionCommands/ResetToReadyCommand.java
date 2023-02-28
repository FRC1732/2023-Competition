package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;

public class ResetToReadyCommand extends CommandBase {
  private RobotContainer robotContainer;
  private PieceMode prevPieceMode;

  public ResetToReadyCommand(RobotContainer robotContainer) {
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
    robotContainer.indexerSubsystem.intake(prevPieceMode);
  }

  @Override
  public void execute() {
    if (prevPieceMode != robotContainer.pieceMode) {
      prevPieceMode = robotContainer.pieceMode;
      if (prevPieceMode == PieceMode.CONE) {
        robotContainer.indexerSubsystem.close();
      } else {
        robotContainer.indexerSubsystem.open();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
