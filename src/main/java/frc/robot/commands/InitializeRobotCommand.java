package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.RobotContainer.ScoringHeight;

public class InitializeRobotCommand extends CommandBase {
  private RobotContainer robotContainer;
  private final PieceMode pieceMode;
  private final ScoringHeight scoringHeight;
  private final Rotation2d startingAngle;

  public InitializeRobotCommand(
      RobotContainer robotContainer,
      PieceMode pieceMode,
      ScoringHeight scoringHeight,
      Rotation2d startingAngle) {
    this.robotContainer = robotContainer;
    this.pieceMode = pieceMode;
    this.scoringHeight = scoringHeight;
    this.startingAngle = startingAngle;
  }

  public void initialize() {
    robotContainer.pieceMode = pieceMode;
    robotContainer.scoringHeight = scoringHeight;
    robotContainer.drivetrainSubsystem.setGyroOffset(startingAngle.getDegrees());
  }

  public void execute() {}

  public void end(boolean interrupted) {}

  public boolean isFinished() {
    return true;
  }
}
