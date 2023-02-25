package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.SwitchToLow;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerSwitchToLowCommand extends CommandBase {
  private RobotStateMachine robotStateMachine;
  private IndexerSubsystem indexerSubsystem;

  public IndexerSwitchToLowCommand(
      RobotContainer robotContainer, RobotStateMachine robotStateMachine) {
    this.indexerSubsystem = robotContainer.indexerSubsystem;
    addRequirements(indexerSubsystem);
  }

  public void initialize() {}

  @Override
  public void execute() {}

  public void end(boolean interrupted) {
    robotStateMachine.fireEvent(new SwitchToLow()); // TODO: make piece detected
  }

  public boolean isFinished() {
    // return indexerSubsytem.hasPiece();
    return true;
  }
}
