package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.HolderSubsystem;

public class SetExtenderStabilizer extends CommandBase {
  private RobotStateMachine robotStateMachine;
  private ElevatorSubsystem elevatorSubsystem;
  private ExtenderSubsystem extenderSubsystem;
  private HolderSubsystem holderSubsystem;
  private boolean state;

  public SetExtenderStabilizer(RobotContainer robotContainer, boolean state) {
    this.extenderSubsystem = robotContainer.extenderSubsystem;
    this.state = state;
    addRequirements(extenderSubsystem);
  }

  public void initialize() {}

  @Override
  public void execute() {
    if (state) extenderSubsystem.engageStablizer();
    else extenderSubsystem.disengageStablizer();
  }

  public void end(boolean interrupted) {
    // robotStateMachine.fireEvent(new FinishScorePressed()); // TODO: make piece detected
  }

  public boolean isFinished() {
    // return intakeSubsytem.hasPiece();
    return true;
  }
}
