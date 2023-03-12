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
  private Constants() {
    throw new IllegalStateException("attempted to instantiate static class");
  }

  public static final double LOOP_PERIOD_SECS = 0.02;

  public static final boolean TUNING_MODE = false;

  public static final boolean DEBUGGING = false;

  public static final String CAN_BUS_NAME = "Oogway";

  public static final double MAX_PRESSURE = 120.0;
  public static final double MIN_PRESSURE = 100.0;

  // #region Subsystem Config

  public static final boolean HARDWARE_CONFIG_HAS_DRIVETRAIN = true;
  public static final boolean HARDWARE_CONFIG_HAS_ELEVATOR = true;
  public static final boolean HARDWARE_CONFIG_HAS_EXTENDER = true;
  public static final boolean HARDWARE_CONFIG_HAS_HOLDER = true;
  public static final boolean HARDWARE_CONFIG_HAS_INDEXER = true;
  public static final boolean HARDWARE_CONFIG_HAS_STATEMACHINE = true;
  public static final boolean HARDWARE_CONFIG_HAS_RGB = true;
  public static final boolean HARDWARE_CONFIG_HAS_BOTH_LIMELIGHTS = true;

  // #endregion Subsystem Config

  // #region CAN IDs

  public static final int CAN_PNEUMATIC_ID = 60;
  public static final int INDEXER_SOLENOID_ID = 0;
  public static final int HOLDER_SOLENOID_ID = 1;

  public static final int ELEVATOR_MAGNETIC_LIMIT_SWITCH_CHANNEL = 0; // FIXME: get port
  public static final int EXTENDER_MAGNETIC_LIMIT_SWITCH = 0; // FIXME: get port

  public static final int ELEVATOR_BASE_MOTOR_ONE_CAN_ID = 50;
  public static final int ELEVATOR_BASE_MOTOR_TWO_CAN_ID = 51;
  public static final int EXTENDER_MOTOR_CAN_ID = 54;
  public static final int INDEXER_ROTATION_CAN_ID = 53;
  public static final int INDEXER_GRABBER_CAN_ID = 56;

  // #endregion CAN IDs

  public static final double MAX_VELOCITY_RADIANS_PER_SECOND = 0.0;
  public static final double MAX_ANGULAR_ACCELERATION = 0;

  public static final double DEADBAND = 0.05;
  public static final double TRAINING_WHEELS = 1.00;

  // #region Indexer Constants

  public static final double INDEXER_INTAKE_SPEED = -0.5;
  public static final double INDEXER_PLACEMENT_SPEED = 0.2;
  public static final double INDEXER_HOLD_SPEED = 0.025;
  public static final double INDEXER_PIECE_DETECTION_CURRENT = 25;
  public static final double INDEXER_TRANSFER_SPEED = 0.1;
  public static final double INDEXER_CONE_POSITION = -115;
  public static final double INDEXER_CUBE_POSITION = -105;
  public static final double INDEXER_SCORING_POSITION = -45;
  public static final double INDEXER_STARTING_POSITION = 0;
  public static final double INDEXER_ARM_DEADBAND = 1;
  public static final double INDEXER_ARM_ROTATE_STALL_SPEED = 0.015;
  public static final double INDEXER_ARM_ROTATE_MAX_SPEED = 1000;
  public static final double INDEXER_ARM_ROTATE_MAX_ACCELERATION = 1800;
  public static final double INDEXER_ARM_P_VALUE = 0.01;
  public static final double INDEXER_ARM_I_VALUE = 0;
  public static final double INDEXER_ARM_D_VALUE = 0;
  public static final double INDEXER_ARM_PID_MAX_OUTPUT = 1;
  public static final double INDEXER_ARM_PID_MIN_OUTPUT = -1;
  public static final double INDEXER_POSITION_CONVERSION_FACTOR = 3.89189189;

  // #endregion

  // #region Elevator Constants

  // public static final int ELEVATOR_TICKS_PER_ROTATION = 8192;
  // public static final int ELEVATOR_MEAUSREMENT_PERIOD_MS = 1;
  public static final double ELEVATOR_INCHES_PER_ROTATION = 0.275199;
  public static final double ELEVATOR_STARTING_POSITION_INCHES = 23.75;
  public static final double ELEVATOR_MIN_POSITION_INCHES = 0;
  public static final double ELEVATOR_MAX_POSITION_INCHES = 35.75;
  public static final double ELEVATOR_CUBE_TRANSFER_POSITION_INCHES = 1;
  public static final double ELEVATOR_CONE_TRANSFER_POSITION_INCHES = 1.5;
  public static final double ELEVATOR_NEUTRAL_POSITION_INCHES = 9;
  public static final double ELEVATOR_MID_CONE_POSITION_INCHES = 23.75;
  public static final double ELEVATOR_HIGH_CONE_POSITION_INCHES = 35.50;
  public static final double ELEVATOR_MAX_SPEED_RPM = 150000;
  public static final double ELEVATOR_MAX_ACCELERATION_RPM2 = 160000;
  public static final double ELEVATOR_DEADBAND = 0.5;
  public static final double ELEVATOR_P_VALUE = 0.00025;
  public static final double ELEVATOR_I_VALUE = 0;
  public static final double ELEVATOR_D_VALUE = 0;
  public static final double ELEVATOR_PID_MAX_OUTPUT = 1;
  public static final double ELEVATOR_PID_MIN_OUTPUT = -1;

  // #endregion

  // #region Extender Constants

  public static final int EXTENDER_TICKS_PER_ROTATION = 42;
  public static final int EXTENDER_MEAUSREMENT_PERIOD_MS = 20;
  public static final double EXTENDER_INCHES_PER_ROTATION = 0.852;
  public static final double EXTENDER_STARTING_POSITION_INCHES = 0;
  public static final double EXTENDER_MIN_POSITION_INCHES = 0;
  public static final double EXTENDER_MAX_POSITION_INCHES = 47.5;
  public static final double EXTENDER_HIGH_CONE_POSITION_INCHES = 53;
  public static final double EXTENDER_MID_CONE_POSITION_INCHES = 34.25;
  public static final double EXTENDER_MAX_SPEED_RPM = 50000;
  public static final double EXTENDER_DEADBAND = 0.5;
  public static final double EXTENDER_MAX_ACCELERATION_RPM2 = 70000;
  public static final double EXTENDER_P_VALUE = 0.000075;
  public static final double EXTENDER_I_VALUE = 0;
  public static final double EXTENDER_D_VALUE = 0;
  public static final double EXTENDER_PID_MAX_OUTPUT = 1;
  public static final double EXTENDER_PID_MIN_OUTPUT = -1;

  // #endregion

  // #region Limelight Constants

  public static final double LIMELIGHT_HEIGHT = 2.229;

  // #endregion
}
