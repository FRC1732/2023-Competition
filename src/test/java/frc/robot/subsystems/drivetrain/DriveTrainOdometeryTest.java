// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static org.junit.jupiter.api.Assertions.*;

import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.swerve.SwerveModule;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Add your docs here. */
public class DriveTrainOdometeryTest {
  private GyroIO gyroIO;
  private SwerveModule flModule;
  private SwerveModule frModule;
  private SwerveModule blModule;
  private SwerveModule brModule;

  private Drivetrain drivetrain;

  @BeforeEach
  public void setup() {
    gyroIO = null;
    flModule = null;
    frModule = null;
    blModule = null;
    brModule = null;

    drivetrain = new Drivetrain(gyroIO, flModule, frModule, blModule, brModule);
  }

  @Test
  public void verifyBasicSetup() {
    assertNotNull(drivetrain);

    drivetrain.getPose();
  }
}
