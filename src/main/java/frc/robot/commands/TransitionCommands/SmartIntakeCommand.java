package frc.robot.commands.TransitionCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.RobotContainer.RobotRotationMode;
import frc.robot.RobotContainer.RobotTranslationMode;
import frc.robot.RobotContainer.ScoringHeight;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.PieceDetectedLow;
import frc.robot.state_machine.events.PieceDetectedMidHigh;

public class SmartIntakeCommand extends CommandBase {
  private RobotContainer robotContainer;
  private RobotStateMachine robotStateMachine;
  private int intakeCount = 0;
  private PieceMode prevPieceMode;

  public SmartIntakeCommand(RobotContainer robotContainer, RobotStateMachine robotStateMachine) {
    this.robotStateMachine = robotStateMachine;
    this.robotContainer = robotContainer;
    addRequirements(robotContainer.indexerSubsystem);
    addRequirements(robotContainer.elevatorSubsystem);
    addRequirements(robotContainer.extenderSubsystem);
    addRequirements(robotContainer.holderSubsystem);
  }

  @Override
  public void initialize() {
    prevPieceMode = robotContainer.pieceMode;
    robotContainer.elevatorSubsystem.setToNeutralPosition();
    robotContainer.extenderSubsystem.goToStartingPosition();
    robotContainer.holderSubsystem.open();
    robotContainer.indexerSubsystem.intake(prevPieceMode);
    robotContainer.robotRotationMode = RobotRotationMode.PIECE_TRACKING;
    robotContainer.robotTranslationMode = RobotTranslationMode.PIECE_TRACKING;
  }

  @Override
  public void execute() {
    if (prevPieceMode != robotContainer.pieceMode) {
      prevPieceMode = robotContainer.pieceMode;
      robotContainer.indexerSubsystem.intake(prevPieceMode);
    }
    if (prevPieceMode == PieceMode.CUBE) {
      robotContainer.indexerSubsystem.grabberIntake(
          robotContainer.drivetrainSubsystem.getPercentMaxSpeed());
    }
  }

  @Override
  public void end(boolean interrupted) {
    robotContainer.robotRotationMode = RobotRotationMode.DRIVER;
    robotContainer.robotTranslationMode = RobotTranslationMode.DRIVER;
    if (interrupted) {
      return;
    }
    robotContainer.rgbStatusSubsytem.capturedGamePiece();
    if (robotContainer.scoringHeight == ScoringHeight.LOW) {
      robotStateMachine.fireEvent(new PieceDetectedLow());
    } else {
      robotStateMachine.fireEvent(new PieceDetectedMidHigh());
    }
  }

  @Override
  public boolean isFinished() {
    if (robotContainer.indexerSubsystem.hasPiece()) {
      intakeCount++;
      return intakeCount > 10;
    }
    intakeCount = 0;
    return false;
  }
}
