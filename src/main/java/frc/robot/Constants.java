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
  public static final double INDEXER_HOLD_SPEED = -0.1;
  public static final double INDEXER_TRANSFER_SPEED = 0.1;
  public static final double INDEXER_CONE_POSITION = 200;
  public static final double INDEXER_CUBE_POSITION = 150;
  public static final double INDEXER_ARM_ROTATE_MAX_SPEED = 0.5;
  public static final double INDEXER_ARM_P_VALUE = 0.001;

  // #endregion

  // #region Elevator Constants

  public static final int ELEVATOR_TICKS_PER_ROTATION = 8192;
  public static final int ELEVATOR_MEAUSREMENT_PERIOD_MS = 20;

  public static final double ELEVATOR_INCHES_PER_ROTATION = 2 * Math.PI;

  // #endregion

  // #region Extender Constants

  // #endregion
}
