// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class DefaultIndexerCommand extends CommandBase {
  IndexerSubsystem indexerSubsystem;
  // private double startingPosition;

  public DefaultIndexerCommand(IndexerSubsystem indexerSubsystem) {
    addRequirements(indexerSubsystem);
    this.indexerSubsystem = indexerSubsystem;
  }

  @Override
  public void initialize() {
    // startingPosition = indexerSubsystem.getArmRotation();
  }

  @Override
  public void execute() {
    /* if (startingPosition != indexerSubsystem.getArmRotation()) {
      if (startingPosition > indexerSubsystem.getArmRotation()) {
        indexerSubsystem.rotateDown();
      } else {
        indexerSubsystem.rotateUp();
      }
    } */
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println(
        "DefaultIntakeCommand - Interrupted [" + (interrupted ? "TRUE" : "FALSE") + "]");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
