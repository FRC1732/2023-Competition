package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.RobotContainer.ScoringHeight;

public class InitializeRobotCommand extends CommandBase {
  private RobotContainer robotContainer;
  private final PieceMode pieceMode;
  private final ScoringHeight scoringHeight;
  private Pose2d startingPose;

  public InitializeRobotCommand(
      RobotContainer robotContainer,
      PieceMode pieceMode,
      ScoringHeight scoringHeight,
      Pose2d startingPose) {
    this.robotContainer = robotContainer;
    this.pieceMode = pieceMode;
    this.scoringHeight = scoringHeight;
    this.startingPose = startingPose;
  }

  public InitializeRobotCommand(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    this.robotContainer.gyro.calibrate();
    this.pieceMode = PieceMode.CONE;
    this.scoringHeight = ScoringHeight.HIGH;
    this.startingPose = new Pose2d(0, 0, Rotation2d.fromRadians(Math.PI));
  }

  public InitializeRobotCommand(RobotContainer robotContainer, Pose2d startingPose) {
    this.robotContainer = robotContainer;
    this.robotContainer.gyro.calibrate();
    this.pieceMode = PieceMode.CONE;
    this.scoringHeight = ScoringHeight.HIGH;
    this.startingPose = startingPose;
  }

  public void initialize() {
    robotContainer.pieceMode = pieceMode;
    robotContainer.scoringHeight = scoringHeight;
    robotContainer.drivetrainSubsystem.resetOdometry(
        CommandFactory.getAllianceCorrectedPose(startingPose));
  }

  public void execute() {}

  public void end(boolean interrupted) {}

  public boolean isFinished() {
    return true;
  }
}
