// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.RobotContainer.RobotRotationMode;
import frc.robot.RobotContainer.RobotTranslationMode;
import frc.robot.RobotContainer.ScoringHeight;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.LimelightObjectDetection;
import frc.robot.subsystems.LimelightScoring;
import frc.robot.subsystems.LimelightScoring.ScoringMode;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import org.littletonrobotics.junction.Logger;

public class TeleopSwervePlus extends CommandBase {
  private final RobotContainer robotContainer;
  private final OperatorInterface oi;

  private Drivetrain drivetrainSubsystem;

  // FIXME: drive train has several PID Controllers already created for use with
  // Autos. Should we
  // just reuse them here?
  private PIDController rotationPidController = null;
  private PIDController translationPidController = null;

  private final double KOWALSKI_2022_P = 13;
  private final double KOWALSKI_2022_I = 0;
  private final double KOWALSKI_2022_D = 1;

  private final double KP = KOWALSKI_2022_P;
  private final double KI = KOWALSKI_2022_I;
  private final double KD = KOWALSKI_2022_D;

  /**
   * Create a new TeleopSwerve command object.
   *
   * @param oi
   * @param robotContainer
   */
  public TeleopSwervePlus(RobotContainer robotContainer, OperatorInterface oi) {
    this.robotContainer = robotContainer;
    this.oi = oi;

    drivetrainSubsystem = robotContainer.drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    rotationPidController = new PIDController(KP, KI, KD);
    rotationPidController.enableContinuousInput(Math.PI * -1, Math.PI);
    translationPidController =
        new PIDController(
            DrivetrainConstants.AUTO_DRIVE_P_CONTROLLER,
            DrivetrainConstants.AUTO_DRIVE_I_CONTROLLER,
            DrivetrainConstants.AUTO_DRIVE_D_CONTROLLER);
  }

  @Override
  public void execute() {
    double xPercentage = oi.getTranslateX();
    double yPercentage = oi.getTranslateY();
    double rotationPercentage = oi.getRotate();

    if (oi.getVisionAssistButton().getAsBoolean()) {
      // make sure we are parsing the JSON when we need it...
      if (robotContainer.robotRotationMode == RobotRotationMode.PIECE_TRACKING) {
        robotContainer.limelightObjectDetectionSubsystem.doDetection();
      } else {
        robotContainer.limelightObjectDetectionSubsystem.stopDetection();
      }

      if (robotContainer.robotRotationMode == RobotRotationMode.DRIVER
          && robotContainer.robotTranslationMode == RobotTranslationMode.DRIVER) {
        drivetrainSubsystem.enableFieldRelative();
      } else {
        drivetrainSubsystem.disableFieldRelative();
      }

      switch (robotContainer.robotRotationMode) {
        case PIECE_TRACKING:
          rotationPercentage = doPieceTrackingRotation(rotationPercentage);
          yPercentage = 0; // lock out side to side
          break;
        case LOCK_TO_ZERO:
          rotationPercentage = doLockToZeroRotation();
          break;
        case DRIVER:
          rotationPidController.reset();
          break;
        default:
          break;
      }

      switch (robotContainer.robotTranslationMode) {
        case SCORE_PIECE:
          yPercentage = doScorePieceTranslation(yPercentage);
          break;
        case DRIVER:
          translationPidController.reset();
          break;
        default:
          break;
      }
    }

    drivetrainSubsystem.drivePercentage(xPercentage, yPercentage, rotationPercentage);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    Logger.getInstance().recordOutput("ActiveCommands/TeleopSwervePlus", false);
  }

  private double doPieceTrackingRotation(double defaultResponse) {
    LimelightObjectDetection ll = robotContainer.limelightObjectDetectionSubsystem;

    if (robotContainer.pieceMode == PieceMode.CONE) {
      if (ll.hasConeTarget()) {
        return rotationPidController.calculate(Math.toRadians(ll.getClosestConeTarget().getX()), 0)
            / Math.PI;
      }
    }

    if (robotContainer.pieceMode == PieceMode.CUBE) {
      if (ll.hasCubeTarget()) {
        return rotationPidController.calculate(Math.toRadians(ll.getClosestCubeTarget().getX()), 0)
            / Math.PI;
      }
    }

    return defaultResponse;
  }

  private double doLockToZeroRotation() {
    // target of 180 degrees facing the driver stations
    return rotationPidController.calculate(
            drivetrainSubsystem.getPose().getRotation().getRadians(), Math.PI)
        / Math.PI;
  }

  private double doScorePieceTranslation(double defaultResponse) {
    LimelightScoring ll = robotContainer.limelightScoringSubSystem;
    ll.setScoringMode(ScoringMode.ReflectiveTape);

    if (robotContainer.scoringHeight != ScoringHeight.LOW && ll.hasTarget()) {
      return translationPidController.calculate(ll.getTx(), 0) / 1.0;
      // divide by some unknown scaling factor to translate position into percentage
    }

    return defaultResponse;
  }
}
