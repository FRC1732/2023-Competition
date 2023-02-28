package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;

public class ReleasePieceFromCarryingCommand extends WaitCommand {
  private RobotContainer robotContainer;
  private PieceMode prevPieceMode;
  private boolean elevatorSetFlag = false;

  public ReleasePieceFromCarryingCommand(RobotContainer robotContainer) {
    super(1.5); // run for 1.5 seconds
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.indexerSubsystem);
    addRequirements(robotContainer.elevatorSubsystem);
    addRequirements(robotContainer.extenderSubsystem);
    addRequirements(robotContainer.holderSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    prevPieceMode = robotContainer.pieceMode;
    robotContainer.extenderSubsystem.goToStartingPosition();
    robotContainer.holderSubsystem.open();
  }

  @Override
  public void execute() {
    // let the holder open before moving
    if (!elevatorSetFlag && m_timer.get() > 0.5) {
      robotContainer.elevatorSubsystem.setToNeutralPosition();
    }

    // prevent changing piece mode
    if (prevPieceMode != robotContainer.pieceMode) {
      robotContainer.pieceMode = prevPieceMode;
    }
  }
}
