package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.state_machine.RobotStateMachine;

public class UnstageGamePieceCommand extends CommandBase {
  private RobotContainer robotContainer;
  private RobotStateMachine robotStateMachine;
  private int intakeCount = 0;
  private PieceMode prevPieceMode;

  public UnstageGamePieceCommand(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.elevatorSubsystem);
    addRequirements(robotContainer.extenderSubsystem);
    addRequirements(robotContainer.holderSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
