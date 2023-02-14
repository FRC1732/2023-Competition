package frc.robot.commands.ElevatorCommands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ElevatorUp extends CommandBase {
    ElevatorSubsystem elevatorSubsystem; 
    private double rotations;
    private double offset;

    public ElevatorUp(ElevatorSubsystem elevatorSubsystem, double rotations, double offset) {
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
        elevatorSubsystem.on();
    }

    public void end(boolean interrupted) {
        System.out.println("ElevatorUpCommand - Interrupted [" + (interrupted ? "TRUE" : "FALSE") + "]");
    }

    @Override
    public boolean isFinished() {
        if (elevatorSubsystem.getPosition() - offset <= rotations) {
            return false;

        }
        return true;
        
    }
}
