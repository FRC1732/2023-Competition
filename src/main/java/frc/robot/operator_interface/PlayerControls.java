// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Class for controlling the robot with two Xbox controllers. */
public class PlayerControls implements OperatorInterface {
  private final CommandJoystick translateJoystick;
  private final CommandJoystick rotateJoystick;
  private final CommandJoystick operatorJoystick;
  private final Trigger[] translateJoystickButtons;
  private final Trigger[] rotateJoystickButtons;
  private final Trigger[] operatorJoystickButtons;

  public PlayerControls(int rotatePort, int translatePort, int operatorPort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);
    operatorJoystick = new CommandJoystick(operatorPort);
    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];
    this.operatorJoystickButtons = new Trigger[9];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
      operatorJoystickButtons[i] = operatorJoystick.button(i);
    }
  }

  @Override
  public double getTranslateX() {
    return -translateJoystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -translateJoystick.getX();
  }

  @Override
  public double getRotate() {
    return -rotateJoystick.getX();
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[6];
  }

  @Override
  public Trigger getResetGyroButton() {
    return rotateJoystickButtons[3];
  }

  @Override
  public Trigger getXStanceButton() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger getIntakeButton() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger getGrabberCubeButton() {
    return rotateJoystickButtons[4];
  }

  @Override
  public Trigger getGrabberConeButton() {
    return rotateJoystickButtons[5];
  }

  @Override
  public Trigger getIndexerRotateUpButton() {
    return translateJoystickButtons[4];
  }

  @Override
  public Trigger getIndexerRotateDownButton() {
    return translateJoystickButtons[5];
  }

  @Override
  public Trigger getGrabberEjectButton() {
    return rotateJoystickButtons[2];
  }

  @Override
  public Trigger getIndexerOpenButton() {
    return translateJoystickButtons[7];
  }

  @Override
  public Trigger getIndexerCloseButton() {
    return translateJoystickButtons[3];
  }
}
