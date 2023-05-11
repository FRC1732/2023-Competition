// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;

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

  public static final String CAN_BUS_NAME = "po";

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
  public static final int EXTENDER_STABLIZER_ID = 2;

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
  public static final double INDEXER_PLACEMENT_SPEED = 0.15;
  public static final double INDEXER_HOLD_SPEED = 0.025;
  public static final double INDEXER_HOLD_UP_SPEED = 0.25;
  public static final double INDEXER_PIECE_DETECTION_CURRENT = 25;
  public static final double INDEXER_TRANSFER_SPEED = 0.2;
  public static final double INDEXER_CONE_POSITION = -115;
  public static final double INDEXER_CONE_AUTO_POSITION = -45;
  public static final double INDEXER_CUBE_POSITION = -110;
  public static final double INDEXER_SCORING_POSITION = -45;
  public static final double INDEXER_STARTING_POSITION = 0;
  public static final double INDEXER_ARM_DEADBAND = 3;
  public static final double INDEXER_ARM_ROTATE_STALL_SPEED = 0.02;
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
  public static final double ELEVATOR_INCHES_PER_ROTATION = 0.275199 * (5.0 / 3.0);
  public static final double ELEVATOR_STARTING_POSITION_INCHES = 23.75;
  public static final double ELEVATOR_MIN_POSITION_INCHES = 0;
  public static final double ELEVATOR_MAX_POSITION_INCHES = 35.75;
  public static final double ELEVATOR_CUBE_TRANSFER_POSITION_INCHES = 1.5;
  public static final double ELEVATOR_CONE_TRANSFER_POSITION_INCHES = 2.0;
  public static final double ELEVATOR_NEUTRAL_POSITION_INCHES = 9;
  public static final double ELEVATOR_MID_CONE_POSITION_INCHES = 23.75;
  public static final double ELEVATOR_HIGH_CONE_POSITION_INCHES = 35.50;
  public static final double ELEVATOR_MAX_SPEED_RPM = 150000;
  public static final double ELEVATOR_MAX_ACCELERATION_RPM2 = 160000;
  public static final double ELEVATOR_DEADBAND = 0.5;
  public static final double ELEVATOR_P_VALUE = 0.0002;
  public static final double ELEVATOR_I_VALUE = 0;
  public static final double ELEVATOR_D_VALUE = 0;
  public static final double ELEVATOR_PID_MAX_OUTPUT = 1;
  public static final double ELEVATOR_PID_MIN_OUTPUT = -1;
  public static final double ELEVATOR_SLOW_DOWN_SPEED = -0.2;

  // #endregion

  // #region Extender Constants

  public static final int EXTENDER_TICKS_PER_ROTATION = 42;
  public static final int EXTENDER_MEAUSREMENT_PERIOD_MS = 20;
  public static final double EXTENDER_INCHES_PER_ROTATION = 0.852;
  public static final double EXTENDER_STARTING_POSITION_INCHES = 0;
  public static final double EXTENDER_MIN_POSITION_INCHES = 0;
  public static final double EXTENDER_MAX_POSITION_INCHES = 47.5;
  public static final double EXTENDER_HIGH_CONE_POSITION_INCHES = 50.25;
  public static final double EXTENDER_MID_CONE_POSITION_INCHES = 32.25;
  public static final double EXTENDER_RESEAT_INCHES = 32.25;
  public static final double EXTENDER_MAX_SPEED_RPM = 180000;
  public static final double EXTENDER_DEADBAND = 0.5;
  public static final double EXTENDER_MAX_ACCELERATION_RPM2 = 500000;
  public static final double EXTENDER_P_VALUE = 0.000075;
  public static final double EXTENDER_I_VALUE = 0;
  public static final double EXTENDER_D_VALUE = 0;
  public static final double EXTENDER_PID_MAX_OUTPUT = 1;
  public static final double EXTENDER_PID_MIN_OUTPUT = -1;

  public static final double SCORING_DISTANCE_TOLERANCE = 36;
  public static final double SCORING_TRANSLATION_TOLERANCE = 8;
  public static final double SCORING_ROTATION_TOLERANCE = 40;
  public static final double SCORING_ABLE_TO_PLACE_TOLERANCE = 5;

  public static final double AUTO_SCORE_SLOW_FORWARD_SPEED = .075;

  public static final double PIECE_DETECTION_P = 7;
  public static final double PIECE_DETECTION_I = 0;
  public static final double PIECE_DETECTION_D = 0;
  public static final double PIECE_DETECTION_DEADZONE = 0;

  public static final double VISION_TRANSLATION_P = .05;
  // #endregion

  // #region Limelight Constants

  public static final double LIMELIGHT_HEIGHT = 2.229;

  // #endregion

  // #region Auto Balance Constants

  public static final double AUTOBALANCE_P_VALUE = 8.5;
  public static final double AUTOBALANCE_I_VALUE = 0;
  public static final double AUTOBALANCE_D_VALUE = 0;

  public static final double AUTOBALANCE_SET_POINT_RADIANS_COMPETITION = Math.PI;
  public static final double AUTOBALANCE_SET_POINT_RADIANS_PROTOBOT = 0;
  public static final double AUTOBALANCE_SET_POINT_RADIANS =
      AUTOBALANCE_SET_POINT_RADIANS_COMPETITION;

  public static final double AUTOBALANCE_LEVEL_TOLERANCE_DEGRESS = 7.0;

  // #endregion

  // #region Auto Waypoints

  // All waypoints assume blue alliance
  // Locations are numbered left->right
  public static final Pose2d ORIGIN = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  public static final Pose2d CONE_NODE_1 = new Pose2d(0.419, 4.977, Rotation2d.fromDegrees(180));
  public static final Pose2d CONE_NODE_1plus =
      new Pose2d(2.419, 4.977, Rotation2d.fromDegrees(180));
  public static final Pose2d CONE_NODE_2 = new Pose2d(0.419, 3.865, Rotation2d.fromDegrees(180));
  public static final Pose2d CONE_NODE_3 = new Pose2d(0.419, 3.309, Rotation2d.fromDegrees(180));
  public static final Pose2d CONE_NODE_4 = new Pose2d(0.419, 2.187, Rotation2d.fromDegrees(180));
  public static final Pose2d CONE_NODE_5 = new Pose2d(0.419, 1.628, Rotation2d.fromDegrees(180));
  public static final Pose2d CONE_NODE_6 = new Pose2d(0.419, 0.512, Rotation2d.fromDegrees(180));
  public static final Pose2d CUBE_NODE_1 =
      new Pose2d(0.419 + 0.25, 4.419 - 0.35, Rotation2d.fromDegrees(180));
  public static final Pose2d CUBE_NODE_2 = new Pose2d(0.419, 2.745, Rotation2d.fromDegrees(180));
  public static final Pose2d CUBE_NODE_3 = new Pose2d(0.419, 1.072, Rotation2d.fromDegrees(180));
  public static final Pose2d NEUTRAL_PIECE_1 = new Pose2d(4.75, 4.85, Rotation2d.fromDegrees(-15));
  public static final Pose2d NEUTRAL_PIECE_2 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  public static final Pose2d NEUTRAL_PIECE_3 = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  public static final Pose2d NEUTRAL_PIECE_4 = new Pose2d(4.833, 0.925, Rotation2d.fromDegrees(0));
  public static final Pose2d CHARGING_STATION_LEFT =
      new Pose2d(2.51, 3.309, Rotation2d.fromDegrees(180));
  public static final Pose2d CHARGING_STATION_CENTER =
      new Pose2d(2.51, 2.745, Rotation2d.fromDegrees(180));
  public static final Pose2d CHARGING_STATION_RIGHT =
      new Pose2d(2.51, 2.187, Rotation2d.fromDegrees(180));
  public static final Pose2d FLAT_LANE_NEAR = new Pose2d(1.089, 4.665, Rotation2d.fromDegrees(180));
  public static final Pose2d FLAT_LANE_FAR = new Pose2d(3.972, 4.665, Rotation2d.fromDegrees(-15));
  public static final Pose2d FLAT_LANE_NEAR_ADJUSTED =
      new Pose2d(1.089, 4.665 - 0.15, Rotation2d.fromDegrees(180));
  public static final Pose2d FLAT_LANE_FAR_ADJUSTED =
      new Pose2d(3.972, 4.665 - 0.15, Rotation2d.fromDegrees(-15));
  public static final Pose2d BUMP_LANE_NEAR = new Pose2d(1.089, 0.747, Rotation2d.fromDegrees(180));
  public static final Pose2d BUMP_LANE_FAR = new Pose2d(3.972, 0.747, Rotation2d.fromDegrees(0));
  public static final List<Translation2d> FLAT_LANE_OUT_WAYPOINTS =
      Arrays.asList(FLAT_LANE_NEAR.getTranslation(), FLAT_LANE_FAR.getTranslation());
  public static final List<Translation2d> FLAT_LANE_IN_WAYPOINTS =
      Arrays.asList(
          FLAT_LANE_FAR_ADJUSTED.getTranslation(), FLAT_LANE_NEAR_ADJUSTED.getTranslation());
  public static final List<Translation2d> BUMP_LANE_OUT_WAYPOINTS =
      Arrays.asList(BUMP_LANE_FAR.getTranslation(), BUMP_LANE_NEAR.getTranslation());
  public static final List<Translation2d> BUMP_LANE_IN_WAYPOINTS =
      Arrays.asList(BUMP_LANE_FAR.getTranslation(), BUMP_LANE_NEAR.getTranslation());

  public static final Pose2d BUMP_START = new Pose2d(3.021, 1.081, Rotation2d.fromDegrees(0));
  public static final Pose2d LAYING_DOWN_4_APPROACH =
      new Pose2d(4.25, 0.917, Rotation2d.fromDegrees(0));
  public static final Pose2d LAYING_DOWN_4 = new Pose2d(5.25, 0.917, Rotation2d.fromDegrees(0));
  public static final Pose2d CONE_DROPOFF = new Pose2d(3.75, 0.662, Rotation2d.fromDegrees(180));
  public static final Pose2d LAYING_DOWN_3_APPROACH_1 =
      new Pose2d(6.357, 1.468, Rotation2d.fromDegrees(180));
  public static final Pose2d LAYING_DOWN_3_APPROACH_2 =
      new Pose2d(6.357, 2.05, Rotation2d.fromDegrees(180));
  public static final Pose2d LAYING_DOWN_3 = new Pose2d(6, 2.05, Rotation2d.fromDegrees(180));
  public static final Pose2d CONE_DROPOFF_APPROACH =
      new Pose2d(3.75, 0.662, Rotation2d.fromDegrees(180));
  public static final Pose2d CONE_PICKUP = new Pose2d(3.45, 0.662, Rotation2d.fromDegrees(180));
  public static final Pose2d CONE_PLACEMENT_APPROACH =
      new Pose2d(0.75, 0.662, Rotation2d.fromDegrees(180));
  public static final Pose2d CONE_PLACEMENT_5 = new Pose2d(0.75, 1.63, Rotation2d.fromDegrees(180));
  public static final Pose2d CONE_PLACEMENT_6 =
      new Pose2d(0.75 - 0.1, 0.513 - .0254 * 4, Rotation2d.fromDegrees(180));
  public static final Pose2d FINAL_CONE_APPROACH =
      new Pose2d(5.7 - .2, 1.05, Rotation2d.fromDegrees(90));
  public static final Pose2d FINAL_CONE_GRAB =
      new Pose2d(5.7 + 0.654, 1.7 + .3, Rotation2d.fromDegrees(90));
  public static final Pose2d BUMP_CENTER_LANE_ENTER =
      new Pose2d(3.25, 0.75 + .025 * 3, Rotation2d.fromDegrees(180));
  public static final Pose2d BUMP_CENTER_LANE_CLOSE =
      new Pose2d(1.75, 0.75 + .025 * 3, Rotation2d.fromDegrees(180));
  public static final Pose2d SCORED_NODE_6 = new Pose2d(0.419, 0.51, Rotation2d.fromDegrees(180));

  public static final List<Translation2d> LAYING_DOWN_4_WAYPOINTS =
      Arrays.asList(LAYING_DOWN_4_APPROACH.getTranslation());
  public static final List<Translation2d> LAYING_DOWN_3_WAYPOINTS =
      Arrays.asList(LAYING_DOWN_3_APPROACH_1.getTranslation());
  public static final List<Translation2d> PICKUP_WAYPOINTS =
      Arrays.asList(LAYING_DOWN_3.getTranslation(), CONE_DROPOFF_APPROACH.getTranslation());
  public static final List<Translation2d> PLACEMENT_5_WAYPOINTS =
      Arrays.asList(CONE_PLACEMENT_APPROACH.getTranslation());
  public static final List<Translation2d> BUMP_CENTER_WAYPOINTS =
      Arrays.asList(
          BUMP_CENTER_LANE_ENTER.getTranslation(), BUMP_CENTER_LANE_CLOSE.getTranslation());
  public static final List<Translation2d> BUMP_CENTER_WAYPOINTS_2 =
      Arrays.asList(FINAL_CONE_APPROACH.getTranslation(), BUMP_CENTER_LANE_ENTER.getTranslation());
  public static final List<Translation2d> REVERSE_BUMP_CENTER_WAYPOINTS =
      Arrays.asList(
          BUMP_CENTER_LANE_CLOSE.getTranslation(),
          BUMP_CENTER_LANE_ENTER.getTranslation(),
          FINAL_CONE_APPROACH.getTranslation());

  // #endregion
}
