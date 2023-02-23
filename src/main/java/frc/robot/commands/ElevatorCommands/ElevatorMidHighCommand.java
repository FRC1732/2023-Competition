package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.PieceDetectedMidHigh;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMidHighCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;

  public ElevatorMidHighCommand() {
    this.elevatorSubsystem = RobotContainer.getInstance().elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  public void initialize() {}

  @Override
  public void execute() {}

  public void end(boolean interrupted) {
    RobotStateMachine.getInstance()
        .fireEvent(new PieceDetectedMidHigh()); // TODO: make piece detected
  }

  public boolean isFinished() {
    // return intakeSubsytem.hasPiece();
    return true;
  }
}
