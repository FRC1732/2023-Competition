// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/** Class for controlling the robot with two Xbox controllers. */
public class PlayerControls implements OperatorInterface {
  private final CommandJoystick translateJoystick;
  private final CommandJoystick rotateJoystick;
  private final CommandJoystick operatorJoystick;
  private final CommandJoystick operatorJoystick2;
  private final Trigger[] translateJoystickButtons;
  private final Trigger[] rotateJoystickButtons;
  private final Trigger[] operatorJoystickButtons;
  private final Trigger[] operatorJoystick2Buttons;

  public PlayerControls(int rotatePort, int translatePort, int operatorPort, int operatorPort2) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);
    operatorJoystick = new CommandJoystick(operatorPort);
    operatorJoystick2 = new CommandJoystick(operatorPort2);
    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];
    this.operatorJoystickButtons = new Trigger[13];
    this.operatorJoystick2Buttons = new Trigger[13];

    bindButtons(translateJoystickButtons, translateJoystick);
    bindButtons(rotateJoystickButtons, rotateJoystick);
    bindButtons(operatorJoystickButtons, operatorJoystick);
    bindButtons(operatorJoystick2Buttons, operatorJoystick2);
  }

  private static void bindButtons(Trigger[] buttons, CommandJoystick joystick) {
    for (int i = 0; i < buttons.length; i++) {
      buttons[i] = joystick.button(i);
    }
  }

  // #region Rotate Joystick

  @Override
  public double getRotate() {
    return modifyAxis(-rotateJoystick.getX());
  }

  @Override
  public Trigger getScoreButton() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger getDeployHolderButton() {
    return rotateJoystickButtons[2];
  }

  // #endregion

  // #region Translate Joystick

  @Override
  public double getTranslateX() {
    return modifyAxis(-translateJoystick.getY());
  }

  @Override
  public double getTranslateY() {
    return modifyAxis(-translateJoystick.getX());
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[6];
  }

  @Override
  public Trigger getResetGyroButton() {
    return translateJoystickButtons[3];
  }

  @Override
  public Trigger getIntakeButton() {
    return translateJoystickButtons[1];
  }

  // #endregion

  // #region Operator Panel

  @Override
  public Trigger getVisionAssistButton() {
    return operatorJoystick2Buttons[11];
  }

  @Override
  public Trigger getXStanceButton() {
    return operatorJoystick2Buttons[10];
  }

  @Override
  public Trigger getCubeModeButton() {
    return operatorJoystick2Buttons[8];
  }

  @Override
  public Trigger getConeModeButton() {
    return operatorJoystick2Buttons[6];
  }

  @Override
  public Trigger getHighGoalButton() {
    return operatorJoystick2Buttons[7];
  }

  @Override
  public Trigger getMiddleGoalButton() {
    return operatorJoystick2Buttons[4];
  }

  @Override
  public Trigger getLowGoalButton() {
    return operatorJoystick2Buttons[3];
  }

  @Override
  public Trigger getHodlerOpenButton() {
    return operatorJoystick2Buttons[12];
  }

  @Override
  public Trigger getAdjustElevatorDownButton() {
    return operatorJoystick2Buttons[2];
  }

  @Override
  public Trigger getAdjustElevatorUpButton() {
    return operatorJoystick2Buttons[1];
  }

  @Override
  public Trigger getIndexerToggleOpenButton() {
    return operatorJoystick2Buttons[9];
  }

  @Override
  public Trigger getIndexerIntakeButton() {
    return operatorJoystickButtons[4];
  }

  @Override
  public Trigger getIndexerEjectButton() {
    return operatorJoystickButtons[3];
  }

  @Override
  public Trigger getIndexerRotateUpButton() {
    return operatorJoystickButtons[1];
  }

  @Override
  public Trigger getIndexerRotateDownButton() {
    return operatorJoystickButtons[2];
  }

  @Override
  public Trigger getIndexerManualOverrideButton() {
    return operatorJoystickButtons[12];
  }

  // #endregion

  /**
   * Squares the specified value, while preserving the sign. This method is used on all joystick
   * inputs. This is useful as a non-linear range is more natural for the driver.
   *
   * @param value
   * @return
   */
  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, Constants.DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);
    value *= Constants.TRAINING_WHEELS;
    return value;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
}
