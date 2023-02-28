package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.FinishScorePressed;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.HolderSubsystem;

public class ScoreWithHolderCommand extends CommandBase {
  private RobotStateMachine robotStateMachine;
  private ElevatorSubsystem elevatorSubsystem;
  private ExtenderSubsystem extenderSubsystem;
  private HolderSubsystem holderSubsystem;

  public ScoreWithHolderCommand(
      RobotContainer robotContainer, RobotStateMachine robotStateMachine) {
    this.holderSubsystem = robotContainer.holderSubsystem;
    this.elevatorSubsystem = robotContainer.elevatorSubsystem;
    this.extenderSubsystem = robotContainer.extenderSubsystem;
    this.robotStateMachine = robotStateMachine;
    addRequirements(elevatorSubsystem);
    addRequirements(extenderSubsystem);
    addRequirements(holderSubsystem);
  }

  public void initialize() {}

  @Override
  public void execute() {}

  public void end(boolean interrupted) {
    robotStateMachine.fireEvent(new FinishScorePressed()); // TODO: make piece detected
  }

  public boolean isFinished() {
    // return intakeSubsytem.hasPiece();
    return true;
  }
}
