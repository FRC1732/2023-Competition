package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.PieceDetectedLow;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCarryLowCommand extends CommandBase {
  private IndexerSubsystem indexerSubsystem;

  public IndexerCarryLowCommand() {
    this.indexerSubsystem = RobotContainer.getInstance().indexerSubsystem;
    addRequirements(indexerSubsystem);
  }

  public void initialize() {}

  @Override
  public void execute() {}

  public void end(boolean interrupted) {
    RobotStateMachine.getInstance().fireEvent(new PieceDetectedLow()); // TODO: make piece detected
  }

  public boolean isFinished() {
    // return indexerSubsytem.hasPiece();
    return true;
  }
}
