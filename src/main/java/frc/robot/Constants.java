// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double LOOP_PERIOD_SECS = 0.02;

  public static final boolean TUNING_MODE = false;

  public static final String CAN_BUS_NAME = "Oogway";

  public static final int CAN_PNEUMATIC_ID = 5; // FIXME: get correct can id

  public static final int INDEXER_SOLENOID_ID = 5; // FIXME: get correct can id

  public static final int INTAKE_CAN_ID = 50;

  public static final int INDEXER_ROTATION_CAN_ID = 51; // FIXME: get correct can id
  public static final int INDEXER_GRABBER_CAN_ID = 52; // FIXME:  get correct can id

  public static final String CAMERA_NAME = "ov9268";

  private static final RobotType ROBOT =
      RobotType.ROBOT_2023_PRESEASON; // TODO: toggle as nessecary
  // private static final RobotType ROBOT = RobotType.ROBOT_2023_COMPETITION;

  public static RobotType getRobot() {
    return ROBOT;
  }

  public static Mode getMode() {
    return Mode.REAL;
  }

  public enum RobotType {
    ROBOT_2023_PRESEASON,
    ROBOT_2023_COMPETITION;
  }

  public enum Mode {
    REAL
  }
}
