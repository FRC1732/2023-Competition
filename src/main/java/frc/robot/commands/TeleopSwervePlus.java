// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class TeleopSwervePlus extends CommandBase {
  private final RobotContainer robotContainer;
  private final RobotStateMachine robotStateMachine;
  private final OperatorInterface oi;

  private Drivetrain drivetrainSubsystem;

  /**
   * Create a new TeleopSwerve command object.
   *
   * @param oi
   * @param robotStateMachine
   * @param robotContainer
   */
  public TeleopSwervePlus(
      RobotContainer robotContainer, RobotStateMachine robotStateMachine, OperatorInterface oi) {
    this.robotContainer = robotContainer;
    this.robotStateMachine = robotStateMachine;
    this.oi = oi;

    drivetrainSubsystem = robotContainer.drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    double xPercentage = oi.getTranslateX();
    double yPercentage = oi.getTranslateY();
    double rotationPercentage = oi.getRotate();

    drivetrainSubsystem.drivePercentage(xPercentage, yPercentage, rotationPercentage);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    Logger.getInstance().recordOutput("ActiveCommands/TeleopSwerve", false);
  }
}
