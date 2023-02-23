package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.ScoringFinishes;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.HolderSubsystem;

public class ResetFromScoringCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private ExtenderSubsystem extenderSubsystem;
  private HolderSubsystem holderSubsystem;

  public ResetFromScoringCommand() {
    this.holderSubsystem = RobotContainer.getInstance().holderSubsystem;
    this.elevatorSubsystem = RobotContainer.getInstance().elevatorSubsystem;
    this.extenderSubsystem = RobotContainer.getInstance().extenderSubsystem;
    addRequirements(elevatorSubsystem);
    addRequirements(extenderSubsystem);
    addRequirements(holderSubsystem);
  }

  public void initialize() {}

  @Override
  public void execute() {}

  public void end(boolean interrupted) {
    RobotStateMachine.getInstance().fireEvent(new ScoringFinishes()); // TODO: make piece detected
  }

  public boolean isFinished() {
    // return intakeSubsytem.hasPiece();
    return true;
  }
}
