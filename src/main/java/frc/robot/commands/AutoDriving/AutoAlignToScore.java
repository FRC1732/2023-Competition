package frc.robot.commands.AutoDriving;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotRotationMode;
import frc.robot.RobotContainer.RobotTranslationMode;
import frc.robot.commands.TeleopSwervePlus;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.ScorePressed;
import frc.robot.subsystems.LimelightScoring;

public class AutoAlignToScore extends CommandBase {
  RobotContainer robotContainer;
  RobotStateMachine robotStateMachine;
  LimelightScoring limelightScoring;

  public AutoAlignToScore(
      RobotContainer robotContainer,
      RobotStateMachine robotStateMachine,
      LimelightScoring limelightScoring) {
    this.robotContainer = robotContainer;
    this.robotStateMachine = robotStateMachine;
    this.limelightScoring = limelightScoring;
  }

  @Override
  public void initialize() {
    robotContainer.robotTranslationMode = RobotTranslationMode.SCORE_PIECE;
    robotContainer.robotRotationMode = RobotRotationMode.SCORE_PIECE;
    TeleopSwervePlus.resetInitialAlignment();
  }

  @Override
  public void end(boolean interrupted) {
    robotContainer.robotRotationMode = RobotRotationMode.DRIVER;
    robotContainer.robotTranslationMode = RobotTranslationMode.DRIVE_FORWARD;
    robotStateMachine.fireEvent(new ScorePressed());
  }

  @Override
  public boolean isFinished() {
    return limelightScoring.isAtInterpDistanceSetpoint();
  }
}
