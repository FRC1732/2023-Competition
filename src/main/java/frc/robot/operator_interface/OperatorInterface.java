// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  // #region Rotate Joystick

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger getScoreButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDeployHolderButton() {
    return new Trigger(() -> false);
  }

  // #endregion

  // #region Translate Joystick

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeButton() {
    return new Trigger(() -> false);
  }

  // #endregion

  // #region Operator Panel

  public default Trigger getVisionAssistButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getCubeModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getConeModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getHighGoalButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getMiddleGoalButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLowGoalButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getHodlerOpenButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getAdjustElevatorDownButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getAdjustElevatorUpButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIndexerToggleOpenButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getHolderPneumaticsButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIndexerEjectButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIndexerRotateUpButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIndexerRotateDownButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIndexerManualOverrideButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getHumanPlayerIntakeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getExtenderStablizerButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getExtenderStablizerDisengageButton() {
    return new Trigger(() -> false);
  }

  // #endregion
}
