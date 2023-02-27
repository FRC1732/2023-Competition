// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RGBStatus extends SubsystemBase {

  private DigitalOutput out0 = new DigitalOutput(0);
  private DigitalOutput out1 = new DigitalOutput(1);
  private DigitalOutput out2 = new DigitalOutput(2);
  private DigitalOutput out3 = new DigitalOutput(3);
  private DigitalOutput out4 = new DigitalOutput(4);
  private ScoreColors scoreColors;
  private GamePiece gamePiece;

  /** Creates a new RGBStatus. */
  public RGBStatus() {
    scoreColors = ScoreColors.NONE;
    gamePiece = GamePiece.NONE;
  }

  @Override
  public void periodic() {
    switch (scoreColors) {
      case HIGH:
        out0.set(true);
        out1.set(true);
        break;
      case LOW:
        out0.set(true);
        out1.set(false);
        break;
      case MEDIUM:
        out0.set(false);
        out1.set(true);
        break;
      case NONE:
      default:
        out0.set(false);
        out1.set(false);
        break;
    }
    switch (gamePiece) {
      case CONE:
        out2.set(false);
        out3.set(true);
        break;
      case CUBE:
        out2.set(true);
        out3.set(false);
        break;
      case NONE:
      default:
        out2.set(false);
        out3.set(false);
        break;
    }
    out4.set(!false);
  }

  // public void hasPiece(boolean gamePiece) {}

  public void off() {
    scoreColors = ScoreColors.NONE;
    gamePiece = GamePiece.NONE;
  }

  public void humanSignals(GamePiece gamePiece) {
    this.gamePiece = gamePiece;
  }

  public void scoringPosition(ScoreColors scoreColors) {
    this.scoreColors = scoreColors;
  }

  public enum ScoreColors {
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
}
