package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.swerve.SwerveModuleConstants;
import java.util.HashMap;

public final class DrivetrainConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private DrivetrainConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final double TRAINING_WHEELS = 1.00;

  public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 40; // module 0
  public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 41;
  public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 42;
  public static double FRONT_LEFT_MODULE_STEER_OFFSET = 41.3;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET_COMPETITION = 41.3;
  public static final double FRONT_LEFT_MODULE_STEER_OFFSET_PROTOBOT = 41.3;

  public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 30; // module 1
  public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 31;
  public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 32;
  public static double FRONT_RIGHT_MODULE_STEER_OFFSET = 281.9;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_COMPETITION = 281.9;
  public static final double FRONT_RIGHT_MODULE_STEER_OFFSET_PROTOBOT = 41.3;

  public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10; // module 2
  public static final int BACK_LEFT_MODULE_STEER_MOTOR = 11;
  public static final int BACK_LEFT_MODULE_STEER_ENCODER = 12;
  public static double BACK_LEFT_MODULE_STEER_OFFSET = 280.1;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET_COMPETITION = 280.1;
  public static final double BACK_LEFT_MODULE_STEER_OFFSET_PROTOBOT = 280.1;

  public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 20; // module 3
  public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 21;
  public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22;
  public static double BACK_RIGHT_MODULE_STEER_OFFSET = 74.7 - 48.954 + 47.901;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET_COMPETITION = 74.7 - 48.954 + 47.901;
  public static final double BACK_RIGHT_MODULE_STEER_OFFSET_PROTOBOT = 74.7 - 48.954 + 47.901;

  /* Kowalski */
  public static final int KOWALSKI_CAN_FRONT_LEFT_MODULE_DRIVE_MOTOR = 11;
  public static final int KOWALSKI_CAN_FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12;
  public static final int KOWALSKI_CAN_BACK_RIGHT_MODULE_DRIVE_MOTOR = 13;
  public static final int KOWALSKI_CAN_BACK_LEFT_MODULE_DRIVE_MOTOR = 14;

  public static final int KOWALSKI_CAN_FRONT_LEFT_MODULE_STEER_MOTOR = 15;
  public static final int KOWALSKI_CAN_FRONT_RIGHT_MODULE_STEER_MOTOR = 16;
  public static final int KOWALSKI_CAN_BACK_RIGHT_MODULE_STEER_MOTOR = 17;
  public static final int KOWALSKI_CAN_BACK_LEFT_MODULE_STEER_MOTOR = 18;

  public static final int KOWALSKI_CAN_FRONT_LEFT_MODULE_STEER_ENCODER = 19;
  public static final int KOWALSKI_CAN_FRONT_RIGHT_MODULE_STEER_ENCODER = 20;
  public static final int KOWALSKI_CAN_BACK_RIGHT_MODULE_STEER_ENCODER = 21;
  public static final int KOWALSKI_CAN_BACK_LEFT_MODULE_STEER_ENCODER = 22;

  public static final double KOWALSKI_COMPETITION_FRONT_LEFT_MODULE_STEER_OFFSET = 139.824;
  public static final double KOWALSKI_COMPETITION_FRONT_RIGHT_MODULE_STEER_OFFSET = 274.609 ;
  public static final double KOWALSKI_COMPETITION_BACK_LEFT_MODULE_STEER_OFFSET = 248.842;
  public static final double KOWALSKI_COMPETITION_BACK_RIGHT_MODULE_STEER_OFFSET = 178.849;

  /**
   * The left-to-right distance between the drivetrain wheels
   *
   * <p>Should be measured from center to center.
   */
  public static final double TRACKWIDTH_METERS = 0.5207;

  /**
   * The front-to-back distance between the drivetrain wheels.
   *
   * <p>Should be measured from center to center.
   */
  public static final double WHEELBASE_METERS = 0.5207;

  public static final double ROBOT_WIDTH_WITH_BUMPERS = 0.89; // meters
  public static final double ROBOT_LENGTH_WITH_BUMPERS = 0.91; // meters

  /*
   * The geometry and coordinate systems can be confusing. Refer to this document
   * for a detailed explanation: https://docs.google.com/document/d/
   * 17dg5cIfqVOlQTTbo2ust4QxTZlUoPNzuBu2oe58Ov84/edit#heading=h.x4ppzp81ed1
   */
  public static final SwerveDriveKinematics KINEMATICS =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          // Front right
          new Translation2d(WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
          // Back left
          new Translation2d(-WHEELBASE_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
          // Back right
          new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0));

  /**
   * The formula for calculating the theoretical maximum velocity is: <Motor free speed RPM> / 60 *
   * <Drive reduction> * <Wheel diameter meters> * pi By default this value is setup for a Mk3
   * standard module using Falcon500s to drive.
   */

  // FIXME: determine maximum velocities empirically

  /**
   * The maximum velocity of the robot in meters per second.
   *
   * <p>This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND =
      6380.0
          / 60.0
          / SwerveModuleConstants.DRIVE_GEAR_RATIO
          * SwerveModuleConstants.WHEEL_CIRCUMFERENCE;

  /**
   * The maximum angular velocity of the robot in radians per second.
   *
   * <p>This is a measure of how fast the robot can rotate in place.
   */
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
      MAX_VELOCITY_METERS_PER_SECOND
          / Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0)
          / 2.0;

  public static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;

  public static final int TIMEOUT_MS = 30;

  public static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 2.0;
  public static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
  public static final double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2.0 * Math.PI;
  public static final double AUTO_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = 2.0 * Math.PI;

  // FIXME: tune PID values for auto paths

  public static final double AUTO_DRIVE_P_CONTROLLER = 6.0;
  public static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  public static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  public static final double AUTO_TURN_P_CONTROLLER = 10.0;
  public static final double AUTO_TURN_I_CONTROLLER = 0.0;
  public static final double AUTO_TURN_D_CONTROLLER = 0.0;

  public static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

  public static final double DEADBAND = 0.1;
}
