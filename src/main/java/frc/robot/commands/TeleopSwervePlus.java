// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
  // Autos. Should we just reuse them here?
  private PIDController rotationPidController = null;
  private PIDController translationPidController = null;

  private final double KOWALSKI_2022_P = 13;
  private final double KOWALSKI_2022_I = 0;
  private final double KOWALSKI_2022_D = 1;

  private double KP = 7; // p8 d0 was good
  private double KI = 0;
  private double KD = 0;
  private double deadzone = 0;

  private static boolean shuffleboardIsSetup = false;

  private GenericEntry kPEntry, kIEntry, kDEntry, deadzoneEntry;

  private final double SLOW_MODE_SCALER = 0.25;

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
    setupShuffleboard();
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
    if (kPEntry != null) {
      double kP = kPEntry.getDouble(Constants.PIECE_DETECTION_P); // p8 d0 was good
      double kI = kIEntry.getDouble(Constants.PIECE_DETECTION_I);
      double kD = kDEntry.getDouble(Constants.PIECE_DETECTION_D);
      double deadzone = deadzoneEntry.getDouble(Constants.PIECE_DETECTION_DEADZONE);
      if (kP != this.KP) {
        this.KP = kP;
        rotationPidController.setP(kP);
      }
      if (kI != this.KI) {
        this.KI = kI;
        rotationPidController.setI(kI);
      }
      if (kD != this.KD) {
        this.KD = kD;
        rotationPidController.setD(kD);
      }
      if (deadzone != this.deadzone) {
        this.deadzone = deadzone;
      }
    }

    drivetrainSubsystem.printModuleDistances();
    double xPercentage = oi.getTranslateX();
    double yPercentage = oi.getTranslateY();
    double rotationPercentage = oi.getRotate();
    if (robotContainer.robotTranslationMode == RobotTranslationMode.DRIVER
        && oi.getIndexerManualOverrideButton().getAsBoolean()) {
      robotContainer.robotTranslationMode = RobotTranslationMode.SLOW_MODE;
    }
    if (robotContainer.robotTranslationMode == RobotTranslationMode.SLOW_MODE
        && !oi.getIndexerManualOverrideButton().getAsBoolean()) {
      robotContainer.robotTranslationMode = RobotTranslationMode.DRIVER;
    }
    if (oi.getVisionAssistButton().getAsBoolean() || DriverStation.isAutonomousEnabled()) {
      // make sure we are parsing the JSON when we need it...
      if (robotContainer.robotRotationMode == RobotRotationMode.PIECE_TRACKING) {
        robotContainer.limelightObjectDetectionSubsystem.doDetection();
      } else {
        robotContainer.limelightObjectDetectionSubsystem.stopDetection();
      }

      // if (robotContainer.robotRotationMode == RobotRotationMode.DRIVER
      //     && robotContainer.robotTranslationMode == RobotTranslationMode.DRIVER) {
      //   drivetrainSubsystem.enableFieldRelative();
      // } else {
      //   drivetrainSubsystem.disableFieldRelative();
      // }

      switch (robotContainer.robotRotationMode) {
        case PIECE_TRACKING:
          rotationPercentage = doPieceTrackingRotation(rotationPercentage);
          break;
        case SCORE_PIECE:
          rotationPercentage = doLockToZeroRotation(); // currently LEDS only
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
          xPercentage = -1 * .1;
          break;
        case DRIVER:
          robotContainer.drivetrainSubsystem.enableFieldRelative();
          translationPidController.reset();
          break;
        case PIECE_TRACKING:
          yPercentage = 0;
          break;
        case SLOW_MODE:
          xPercentage = SLOW_MODE_SCALER * xPercentage;
          yPercentage = SLOW_MODE_SCALER * yPercentage;
          break;
        case AUTO_PIECE_TRACKING:
          xPercentage = 0.15;
          yPercentage = 0;
          robotContainer.drivetrainSubsystem.disableFieldRelative();
          System.out.println("WE SHOULD BE DRIVING FORWARD");
          break;
        default:
          break;
      }
    } else {
      if (robotContainer.robotTranslationMode == RobotTranslationMode.SLOW_MODE) {
        xPercentage = SLOW_MODE_SCALER * xPercentage;
        yPercentage = SLOW_MODE_SCALER * yPercentage;
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

    if (robotContainer.pieceMode == PieceMode.CONE && ll.hasConeTarget()) {
      double val =
          rotationPidController.calculate(Math.toRadians(ll.getClosestConeTarget().getX()), 0)
              / Math.PI
              * 0.6;
      if (Math.abs(val) < deadzone) {
        return 0;
      }
      return val;
    }

    if (robotContainer.pieceMode == PieceMode.CUBE && ll.hasCubeTarget()) {
      double val =
          rotationPidController.calculate(Math.toRadians(ll.getClosestCubeTarget().getX()), 0)
              / Math.PI
              * 0.6;
      if (Math.abs(val) < deadzone) {
        return 0;
      }
      return val;
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

  private void setupShuffleboard() {
    if (!shuffleboardIsSetup) {
      shuffleboardIsSetup = true;
      System.out.println("Hello");
      ShuffleboardTab tab;
      tab = Shuffleboard.getTab("Piece Detection");
      // if (true) {
      kPEntry =
          tab.add("PieceDetect_P", Constants.PIECE_DETECTION_P)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kIEntry =
          tab.add("PieceDetect_I", Constants.PIECE_DETECTION_I)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      kDEntry =
          tab.add("PieceDetect_D", Constants.PIECE_DETECTION_D)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      deadzoneEntry =
          tab.add("PieceDetect_Deadzone", Constants.PIECE_DETECTION_DEADZONE)
              .withWidget(BuiltInWidgets.kTextView)
              .getEntry();
      // }
    }
  }
}
