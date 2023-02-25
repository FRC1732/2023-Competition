package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.IntakePressed;
import frc.robot.subsystems.IndexerSubsystem;

public class SmartIntakeCommand extends CommandBase {
  private RobotStateMachine robotStateMachine;
  private IndexerSubsystem indexerSubsystem;

  public SmartIntakeCommand(RobotContainer robotContainer, RobotStateMachine robotStateMachine) {
    this.indexerSubsystem = robotContainer.indexerSubsystem;
    addRequirements(indexerSubsystem);
  }

  public void initialize() {}

  @Override
  public void execute() {}

  public void end(boolean interrupted) {
    robotStateMachine.fireEvent(new IntakePressed()); // TODO: make piece detected
  }

  public boolean isFinished() {
    // return intakeSubsytem.hasPiece();
    return true;
  }
}
