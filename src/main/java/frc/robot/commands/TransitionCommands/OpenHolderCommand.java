package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;

public class OpenHolderCommand extends WaitCommand {
  private RobotContainer robotContainer;
  private PieceMode prevPieceMode;

  public OpenHolderCommand(RobotContainer robotContainer) {
    super(.2); // run for .5 seconds
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.holderSubsystem);
  }

  @Override
  public void initialize() {
    prevPieceMode = robotContainer.pieceMode;
    m_timer.reset();
    m_timer.start();
    robotContainer.holderSubsystem.open();
  }

  @Override
  public void execute() {
    // prevent changing piece mode
    if (prevPieceMode != robotContainer.pieceMode) {
      robotContainer.pieceMode = prevPieceMode;
    }
  }
}
