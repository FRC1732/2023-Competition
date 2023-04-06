package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class DefaultExtenderCommand extends CommandBase {
  public ExtenderSubsystem extenderSubsystem;

  public DefaultExtenderCommand(ExtenderSubsystem extenderSubsystem) {
    addRequirements(extenderSubsystem);
    this.extenderSubsystem = extenderSubsystem;
  }
}
