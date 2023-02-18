package frc.robot.commands.ExtenderCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderOutCommand extends CommandBase{
    boolean highMid;
    ExtenderSubsystem extenderSubsystem;
    TrapezoidProfile profilePIDController;
    TrapezoidProfile.Constraints profileConstraints;

    public ExtenderOutCommand(boolean highMid, ExtenderSubsystem extenderSubsystem) {
        addRequirements(extenderSubsystem);
        this.highMid = highMid;
        this.extenderSubsystem = extenderSubsystem;
        profileConstraints = new TrapezoidProfile.Constraints(0, 0);
    }
}
