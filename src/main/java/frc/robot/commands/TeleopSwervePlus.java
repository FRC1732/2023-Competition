// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.RobotRototionMode;
import frc.robot.RobotContainer.RobotTranslationMode;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class TeleopSwervePlus extends CommandBase {
  private final RobotContainer robotContainer;
  private final OperatorInterface oi;

  private Drivetrain drivetrainSubsystem;

  /**
   * Create a new TeleopSwerve command object.
   *
   * @param oi
   * @param robotStateMachine
   * @param robotContainer
   */
  public TeleopSwervePlus(RobotContainer robotContainer, OperatorInterface oi) {
    this.robotContainer = robotContainer;
    this.oi = oi;

    drivetrainSubsystem = robotContainer.drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    double xPercentage = oi.getTranslateX();
    double yPercentage = oi.getTranslateY();
    double rotationPercentage = oi.getRotate();

    if (oi.getVisionAssistButton().getAsBoolean()) {
      // make stuff happen here

      if (robotContainer.robotRototionMode != RobotRototionMode.DRIVER) {
        switch (robotContainer.robotRototionMode) {
          case PIECE_TRACKING:
            rotationPercentage = doPieceTrackingRotation();
            break;
          case LOCK_TO_ZERO:
            rotationPercentage = doLockToZeroRotation();
            break;
          default:
            break;
        }
      }

      if (robotContainer.robotTranslationMode != RobotTranslationMode.DRIVER) {
        switch (robotContainer.robotTranslationMode) {
          case SCORE_PIECE:
            yPercentage = doScorePieceTranslation();
            break;
          default:
            break;
        }
      }
    }

    drivetrainSubsystem.drivePercentage(xPercentage, yPercentage, rotationPercentage);
  }

  private double doPieceTrackingRotation() {
    return 0;
  }

  private double doLockToZeroRotation() {
    return 0;
  }

  private double doScorePieceTranslation() {
    return 0;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", false);
  }
}
