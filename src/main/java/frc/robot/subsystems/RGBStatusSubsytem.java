// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.RobotContainer.RobotRotationMode;
import frc.robot.RobotContainer.ScoringHeight;

public class RGBStatusSubsytem extends SubsystemBase {
  /*-
   * Outputs 0 and 1 set the scoring height.
   *   Low Score - Bit 0
   *   Mid Score - Bit 1
   *   High Score - Bits 0 and 1
   *
   * Outputs 2 amd 3 set the game piece
   *   Cone - Bit 3
   *   Cube - Bit 2
   *
   * Output 4 is vision on/off
   *   Vision on/off - Bit 4
   *
   * Special modes fill in around whats left
   *   Idle Mode - All bits offs
   *   Capture Game Piece - Bits 4, 3, and 2
   *   Scoring Target Ready - Combination of Scoring Height, no game piece and vision bit on
   *
   */

  private DigitalOutput out0 = new DigitalOutput(1);
  private DigitalOutput out1 = new DigitalOutput(2);
  private DigitalOutput out2 = new DigitalOutput(3);
  private DigitalOutput out3 = new DigitalOutput(4);
  private DigitalOutput out4 = new DigitalOutput(5);

  private ScoreHeight scoreHeight;
  private GamePiece gamePiece;

  private Timer timer;
  private SpecialMode specialMode;
  private double targetElapsedTimeSeconds;

  private RobotContainer robotContainer;

  private boolean hasBeenEnabled = false;
  private boolean previousDriverstationState;

  /** Creates a new RGBStatus. */
  public RGBStatusSubsytem(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    scoreHeight = ScoreHeight.NONE;
    gamePiece = GamePiece.NONE;
    specialMode = SpecialMode.NONE;
    targetElapsedTimeSeconds = 0;
    timer = new Timer();
    previousDriverstationState = false;
  }

  @Override
  public void periodic() {
    if (robotContainer.pieceMode == PieceMode.CUBE) {
      gamePiece = GamePiece.CUBE;
    } else {
      gamePiece = GamePiece.CONE;
    }

    if (robotContainer.scoringHeight == ScoringHeight.HIGH) {
      scoreHeight = ScoreHeight.HIGH;
    } else if (robotContainer.scoringHeight == ScoringHeight.MEDIUM) {
      scoreHeight = ScoreHeight.MEDIUM;
    } else {
      scoreHeight = ScoreHeight.LOW;
    }

    if (!hasBeenEnabled && DriverStation.isEnabled()) {
      hasBeenEnabled = true;
    }

    if (hasBeenEnabled && DriverStation.isDisabled()) {
      specialMode = SpecialMode.GAME_IDLE_MODE;
      targetElapsedTimeSeconds = 0;
    }

    if (hasBeenEnabled && DriverStation.isEnabled()) {
      hasBeenEnabled = true;
    }

    if (previousDriverstationState != DriverStation.isEnabled()) {
      previousDriverstationState = DriverStation.isEnabled();
      if (DriverStation.isEnabled()) {
        specialMode = SpecialMode.NONE;
        targetElapsedTimeSeconds = 0;
      } else {
        specialMode = SpecialMode.GAME_IDLE_MODE;
        targetElapsedTimeSeconds = 0;
      }
    }

    if (robotContainer.robotRotationMode == RobotRotationMode.SCORE_PIECE){
      if( robotContainer.limelightScoringSubSystem.isAligned()){
        specialMode = SpecialMode.SCORING_POSITION_READY;
        targetElapsedTimeSeconds = 0;
      } else {
        specialMode = SpecialMode.NONE;
        targetElapsedTimeSeconds = 0;
      }
    } else if (specialMode == SpecialMode.SCORING_POSITION_READY) {
      specialMode = SpecialMode.NONE;
      targetElapsedTimeSeconds = 0;
    }

    // Invert the digital sigs; HIGH is 0, LOW is 1
    if (specialMode != SpecialMode.NONE) {
      if (targetElapsedTimeSeconds > 0 && timer.hasElapsed(targetElapsedTimeSeconds)) {
        specialMode = SpecialMode.NONE;
        timer.stop();
      } else {
        switch (specialMode) {
          case GAME_PIECE_CAPTURED:
            // bits 4, 3, 2 -- 28
            out0.set(!false);
            out1.set(!false);
            out2.set(!true);
            out3.set(!true);
            out4.set(!true);
            break;

          case GAME_IDLE_MODE:
            // all bits off
            out0.set(!false);
            out1.set(!false);
            out2.set(!false);
            out3.set(!false);
            out4.set(!false);
            break;

          case SCORING_POSITION_READY:
            setScoreHeightBits();

            out2.set(!false);
            out3.set(!false);
            out4.set(!true);
            break;

          case NONE:
          default:
            break;
        }
      }
    }

    if (specialMode == SpecialMode.NONE) {
      setScoreHeightBits();
      setGamePieceBits();
      out4.set(!robotContainer.isVisionOn());
    }
  }

  private void setGamePieceBits() {
    switch (gamePiece) {
      case CONE: // +8
        out2.set(!false);
        out3.set(!true);
        break;

      case CUBE: // +4
        out2.set(!true);
        out3.set(!false);
        break;

      case NONE:
      default:
        out2.set(!false);
        out3.set(!false);
        break;
    }
  }

  private void setScoreHeightBits() {
    switch (scoreHeight) {
      case HIGH: // 3
        out0.set(!true);
        out1.set(!true);
        break;

      case LOW: // 1
        out0.set(!true);
        out1.set(!false);
        break;

      case MEDIUM: // 2
        out0.set(!false);
        out1.set(!true);
        break;

      case NONE:
      default:
        out0.set(!false);
        out1.set(!false);
        break;
    }
  }

  public void capturedGamePiece() {
    timer.reset();
    timer.start();
    targetElapsedTimeSeconds = 1.5;
    specialMode = SpecialMode.GAME_PIECE_CAPTURED;
    System.out.println("RGB - Capture Game Piece");
  }

  public void off() {
    scoreHeight = ScoreHeight.NONE;
    gamePiece = GamePiece.NONE;
    specialMode = SpecialMode.NONE;
    timer.stop();
  }

  public void humanSignals(GamePiece gamePiece) {
    this.gamePiece = gamePiece;
  }

  public void scoringPosition(ScoreHeight scoreColors) {
    this.scoreHeight = scoreColors;
  }

  public enum ScoreHeight {
    HIGH,
    MEDIUM,
    LOW,
    NONE;
  }

  public enum GamePiece {
    CONE,
    CUBE,
    NONE;
  }

  protected enum SpecialMode {
    NONE,
    GAME_PIECE_CAPTURED,
    GAME_IDLE_MODE,
    SCORING_POSITION_READY;
  }
}
