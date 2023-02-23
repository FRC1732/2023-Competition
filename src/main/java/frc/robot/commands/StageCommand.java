package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.ScorePressed;
import frc.robot.subsystems.ElevatorSubsystem;

public class StageCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;

  public StageCommand() {
    this.elevatorSubsystem = RobotContainer.getInstance().elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  public void initialize() {}

  @Override
  public void execute() {}

  public void end(boolean interrupted) {
    RobotStateMachine.getInstance().fireEvent(new ScorePressed()); // TODO: make piece detected
  }

  public boolean isFinished() {
    // return intakeSubsytem.hasPiece();
    return true;
  }
}
