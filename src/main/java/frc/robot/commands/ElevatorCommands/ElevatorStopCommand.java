package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorStopCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;

  public ElevatorStopCommand(ElevatorSubsystem elevatorSubsystem) {
    addRequirements(elevatorSubsystem);
    this.elevatorSubsystem = elevatorSubsystem;
  }

  public void initialize() {}

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    elevatorSubsystem.off();
  }

  public void end(boolean interrupted) {}

  public boolean isFinished() {
    return true;
  }
}
