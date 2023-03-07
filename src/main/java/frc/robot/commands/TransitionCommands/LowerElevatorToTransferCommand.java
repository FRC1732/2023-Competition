package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;

public class LowerElevatorToTransferCommand extends CommandBase {
  private RobotContainer robotContainer;
  private PieceMode prevPieceMode;

  public LowerElevatorToTransferCommand(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.indexerSubsystem);
    addRequirements(robotContainer.elevatorSubsystem);
    addRequirements(robotContainer.extenderSubsystem);
    addRequirements(robotContainer.holderSubsystem);
  }

  @Override
  public void initialize() {
    prevPieceMode = robotContainer.pieceMode;
    robotContainer.elevatorSubsystem.goToTransferPosition(prevPieceMode);
    robotContainer.extenderSubsystem.goToStartingPosition();
    robotContainer.holderSubsystem.open();
    robotContainer.indexerSubsystem.pushAgainstHardstop();
  }

  @Override
  public void execute() {
    if (prevPieceMode != robotContainer.pieceMode) {
      prevPieceMode = robotContainer.pieceMode;
      if (prevPieceMode == PieceMode.CONE) {
        robotContainer.indexerSubsystem.close();
      } else {
        robotContainer.indexerSubsystem.open();
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    robotContainer.indexerSubsystem.grabberOff();
    robotContainer.indexerSubsystem.rotateOff();
    robotContainer.holderSubsystem.open();
  }

  @Override
  public boolean isFinished() {
    return robotContainer.elevatorSubsystem.isAtSetpoint();
  }
}
