package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorUpCommand extends CommandBase {
  ElevatorSubsystem elevatorSubsystem;
  private double rotations;
  private double offset;

  public ElevatorUpCommand(ElevatorSubsystem elevatorSubsystem, double rotations, double offset) {
    addRequirements(elevatorSubsystem);
    this.rotations = rotations;
    this.elevatorSubsystem = elevatorSubsystem;
    this.offset = offset;
  }

  @Override
  public void initialize() {
    offset = elevatorSubsystem.getPosition();
  }

  public void execute() {
    elevatorSubsystem.goUp();
  }

  public void end(boolean interrupted) {
    System.out.println(
        "ElevatorUpCommand - Interrupted [" + (interrupted ? "TRUE" : "FALSE") + "]");
  }

  @Override
  public boolean isFinished() {
    if (elevatorSubsystem.getPosition() - offset <= rotations) {
      return false;
    }
    return true;
  }
}
