// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightScoring;
import frc.robot.subsystems.LimelightScoring.ScoringMode;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.util.function.DoubleSupplier;

public class DriveWithReflectiveAlign extends CommandBase {
  private final Drivetrain drivetrain;
  private final LimelightScoring limelightScoring;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  private PIDController thetaController;

  /** Creates a new DriveWithReflectiveAlign. */
  public DriveWithReflectiveAlign(
      Drivetrain drivetrain,
      LimelightScoring limelightScoring,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.limelightScoring = limelightScoring;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;

    addRequirements(drivetrain);

    thetaController = new PIDController(0, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelightScoring.setScoringMode(ScoringMode.ReflectiveTape);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetX = limelightScoring.getTx();

    double xPercentage = translationXSupplier.getAsDouble();
    double yPercentage = translationYSupplier.getAsDouble();

    double rotationPercentage = 0.0;

    drivetrain.drivePercentage(xPercentage, yPercentage, rotationPercentage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
