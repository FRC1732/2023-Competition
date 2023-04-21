package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ExtenderReseatCommmand extends CommandBase {
  private RobotContainer robotContainer;

  public ExtenderReseatCommmand(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.extenderSubsystem);
  }

  @Override
  public void initialize() {
    robotContainer.extenderSubsystem.goToReseat();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    robotContainer.extenderSubsystem.goToStartingPosition();
  }

  @Override
  public boolean isFinished() {
    return robotContainer.extenderSubsystem.isAtSetpoint();
  }
}
