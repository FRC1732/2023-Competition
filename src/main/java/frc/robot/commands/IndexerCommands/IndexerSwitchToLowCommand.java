package frc.robot.commands.IndexerCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.SwitchToLow;
import frc.robot.subsystems.IntakeSubsystem;

public class IndexerSwitchToLowCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;

  public IndexerSwitchToLowCommand() {
    this.intakeSubsystem = RobotContainer.getInstance().intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  public void initialize() {}

  @Override
  public void execute() {}

  public void end(boolean interrupted) {
    RobotStateMachine.getInstance().fireEvent(new SwitchToLow()); // TODO: make piece detected
  }

  public boolean isFinished() {
    // return intakeSubsytem.hasPiece();
    return true;
  }
}
