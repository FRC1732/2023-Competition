// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HolderSubsystem;

public class DefaultHolderCommand extends CommandBase {
    HolderSubsystem holderSubsystem; 

    public DefaultHolderCommand(HolderSubsystem holderSubsystem) {
        addRequirements(holderSubsystem);
        this.holderSubsystem = holderSubsystem;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DefaultHolderCommand - Interrupted [" + (interrupted ? "TRUE" : "FALSE") + "]");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}