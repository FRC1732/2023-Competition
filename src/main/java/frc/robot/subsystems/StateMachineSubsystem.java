// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.state_machine.RobotStateMachine;
import org.littletonrobotics.junction.Logger;

public class StateMachineSubsystem extends SubsystemBase {
  private RobotStateMachine robotStateMachine;

  /**
   * The purpose of this subsystem is to tie the state machine to a periodic so that we can have
   * periodic level logging. If this proves not helpful, then the default command for this subsystem
   * can be unscheduled to disable this logging.
   */
  public StateMachineSubsystem(RobotStateMachine stateMachine) {
    robotStateMachine = stateMachine;
    setupShuffleboard();
  }

  @Override
  public void periodic() {
    Logger.getInstance()
        .recordOutput("StateMachine/currentState", robotStateMachine.getCurrentState());
    Logger.getInstance().recordOutput("StateMachine/lastEvent", robotStateMachine.getLastEvent());
    Logger.getInstance()
        .recordOutput("StateMachine/lastTransition", robotStateMachine.getLastTransition());
  }

  private void setupShuffleboard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab("StateMachine");
    tab.addString("State", () -> robotStateMachine.getCurrentState());
    tab.addString("Last Event", () -> robotStateMachine.getLastEvent());
    tab.addString("Last Transition", () -> robotStateMachine.getLastEvent());
    }
}
