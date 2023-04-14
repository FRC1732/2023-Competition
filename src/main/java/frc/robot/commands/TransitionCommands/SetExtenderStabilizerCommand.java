package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExtenderSubsystem;

public class SetExtenderStabilizerCommand extends CommandBase {
  private ExtenderSubsystem extenderSubsystem;
  private boolean state;

  public SetExtenderStabilizerCommand(RobotContainer robotContainer, boolean state) {
    this.extenderSubsystem = robotContainer.extenderSubsystem;
    this.state = state;
    addRequirements(extenderSubsystem);
  }

  public void initialize() {
    if (state) extenderSubsystem.engageStablizer();
    else extenderSubsystem.disengageStablizer();
  }
}
