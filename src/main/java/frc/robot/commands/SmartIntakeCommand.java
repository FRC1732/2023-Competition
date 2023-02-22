package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.IntakePressed;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class SmartIntakeCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;

  public SmartIntakeCommand() {
    this.intakeSubsystem = RobotContainer.getInstance().intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  public void initialize() {

  }

  @Override
  public void execute() {
  }
  

  public void end(boolean interrupted) {
    RobotStateMachine.getInstance().fireEvent(new IntakePressed()); //TODO: make piece detected
  }

  public boolean isFinished() {
    //return intakeSubsytem.hasPiece();
    return true;
  }
}