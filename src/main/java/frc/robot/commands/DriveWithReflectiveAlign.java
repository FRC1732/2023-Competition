// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightScoring;
import frc.robot.subsystems.LimelightScoring.ScoringMode;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import java.util.function.DoubleSupplier;

public class DriveWithReflectiveAlign extends CommandBase {
  private final double KOWALSKI_2022_P = 13;
  private final double KOWALSKI_2022_I = 0;
  private final double KOWALSKI_2022_D = 1;

  private final double KP = KOWALSKI_2022_P;
  private final double KI = KOWALSKI_2022_I;
  private final double KD = KOWALSKI_2022_D;

  private final Drivetrain drivetrain;
  private final LimelightScoring limelightScoring;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;

  private ProfiledPIDController thetaController;

  /** Creates a new DriveWithReflectiveAlign. */
  public DriveWithReflectiveAlign(
      Drivetrain drivetrain,
      LimelightScoring limelightScoring,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {

    this.drivetrain = drivetrain;
    this.limelightScoring = limelightScoring;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;

    addRequirements(drivetrain);

    var profileConstraints =
        new TrapezoidProfile.Constraints(
            DrivetrainConstants.AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
            DrivetrainConstants.AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

    thetaController = new ProfiledPIDController(KP, KI, KD, profileConstraints);
    thetaController.enableContinuousInput(Math.PI * -1, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelightScoring.setScoringMode(ScoringMode.ReflectiveTape);
    thetaController.reset(limelightScoring.getTx()); // FIXME: is Tx available yet?
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetX = 0;
    if (limelightScoring.hasTarget()) {
      targetX = limelightScoring.getTx();
    }

    double xPercentage = translationXSupplier.getAsDouble();
    double yPercentage = translationYSupplier.getAsDouble();

    double rotationPercentage = thetaController.calculate(targetX, 0);

    drivetrain.drivePercentage(xPercentage, yPercentage, rotationPercentage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // FIXME: exit when no target or define some other behavior
    if (!limelightScoring.hasTarget()) return true;

    // keep holding alignment while command is active.
    return false;
  }
}
