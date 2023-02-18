package frc.robot.commands.ExtenderCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderOutCommand extends CommandBase{
    boolean highLow;
    ExtenderSubsystem extenderSubsystem;

    public ExtenderOutCommand(boolean highLow, ExtenderSubsystem extenderSubsystem) {
        addRequirements(extenderSubsystem);
        this.highLow = highLow;
        this.extenderSubsystem = extenderSubsystem;
    }
}
