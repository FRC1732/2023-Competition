package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.SwitchToLow;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.HolderSubsystem;

public class StagedToLowCommand extends CommandBase {
  private RobotStateMachine robotStateMachine;
  private ElevatorSubsystem elevatorSubsystem;
  private HolderSubsystem holderSubsystem;

  public StagedToLowCommand(RobotContainer robotContainer, RobotStateMachine robotStateMachine) {
    this.elevatorSubsystem = robotContainer.elevatorSubsystem;
    this.holderSubsystem = robotContainer.holderSubsystem;
    this.robotStateMachine = robotStateMachine;
    addRequirements(elevatorSubsystem);
    addRequirements(holderSubsystem);
  }

  public void initialize() {}

  @Override
  public void execute() {}

  public void end(boolean interrupted) {
    robotStateMachine.fireEvent(new SwitchToLow()); // TODO: make piece detected
  }

  public boolean isFinished() {
    // return intakeSubsytem.hasPiece();
    return true;
  }
}
