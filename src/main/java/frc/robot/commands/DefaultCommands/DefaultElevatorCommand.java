package frc.robot.commands.DefaultCommands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevatorCommand extends CommandBase { 
    private ElevatorSubsystem elevatorSubsystem;
    public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem) { 
        addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }
    public void initialize() {

    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
        elevatorSubsystem.goDown();
    } 
    public void end(boolean interrupted) {
        
    }
    public boolean isFinished() {
      return elevatorSubsystem.getMagLimitSwitch();
    } 
}
