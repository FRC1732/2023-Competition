// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIoADIS16470;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoDriving.AutoAlignToScore;
import frc.robot.commands.AutoDriving.SwerveToWaypointCommand;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.DefaultCommands.DefaultExtenderCommand;
import frc.robot.commands.DefaultCommands.DefaultLimelightObjectDectionCommand;
import frc.robot.commands.DefaultCommands.DefaultLimelightScoringDectionCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.InitializeRobotCommand;
import frc.robot.commands.TeleopSwervePlus;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.state_machine.RobotStateMachine;
import frc.robot.state_machine.events.IntakePressed;
import frc.robot.state_machine.events.IntakeReleased;
import frc.robot.state_machine.events.ScorePressed;
import frc.robot.state_machine.events.SwitchToLow;
import frc.robot.state_machine.events.SwitchToMidHigh;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightObjectDetection;
import frc.robot.subsystems.LimelightScoring;
import frc.robot.subsystems.LimelightScoring.ScoringMode;
import frc.robot.subsystems.RGBStatusSubsytem;
import frc.robot.subsystems.StateMachineSubsystem;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};
  private RobotStateMachine robotStateMachine;

  public Drivetrain drivetrainSubsystem;
  public IndexerSubsystem indexerSubsystem;
  public HolderSubsystem holderSubsystem;
  public ExtenderSubsystem extenderSubsystem;
  public StateMachineSubsystem stateMachineSubsystem;
  public ElevatorSubsystem elevatorSubsystem;
  public RGBStatusSubsytem rgbStatusSubsytem;
  public LimelightObjectDetection limelightObjectDetectionSubsystem;
  public LimelightScoring limelightScoringSubSystem;

  public SwerveModule flModule;
  public SwerveModule frModule;
  public SwerveModule blModule;
  public SwerveModule brModule;
  public GyroIO gyro;
  private GyroIoADIS16470 adis16470Gyro;

  public enum PieceMode {
    CONE,
    CUBE
  }

  public enum ScoringHeight {
    LOW,
    MEDIUM,
    HIGH
  }

  public enum RobotTranslationMode {
    DRIVER,
    SCORE_PIECE,
    PIECE_TRACKING,
    AUTO_PIECE_TRACKING,
    DRIVE_FORWARD
  }

  public enum RobotRotationMode {
    DRIVER,
    PIECE_TRACKING,
    SCORE_PIECE
  }

  // public boolean visionEnabled = true;

  public PieceMode pieceMode = PieceMode.CONE;
  public ScoringHeight scoringHeight = ScoringHeight.HIGH;
  public RobotTranslationMode robotTranslationMode = RobotTranslationMode.DRIVER;
  public RobotRotationMode robotRotationMode = RobotRotationMode.DRIVER;
  public boolean UseAutoAlign = true;
  public boolean collectAtHumanPlayer = false;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to
  // ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDriveTrain();
    defineSubsystems();

    // disable all telemetry in the LiveWindow to reduce the processing during each
    // iteration
    LiveWindow.disableAllTelemetry();
    UseAutoAlign = true;
    robotStateMachine = new RobotStateMachine(this);
    stateMachineSubsystem = new StateMachineSubsystem(robotStateMachine);
    updateOI();
    configureDefaultCommands();
    configureAutoCommands();
  }

  private void configureDriveTrain() {
    adis16470Gyro = new GyroIoADIS16470();
    gyro = adis16470Gyro;

    flModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                0,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET),
            0,
            MAX_VELOCITY_METERS_PER_SECOND);

    frModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                1,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET),
            1,
            MAX_VELOCITY_METERS_PER_SECOND);

    blModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                2,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET),
            2,
            MAX_VELOCITY_METERS_PER_SECOND);

    brModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                3,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET),
            3,
            MAX_VELOCITY_METERS_PER_SECOND);

    drivetrainSubsystem = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    /*-
     * Set up the default command for the drivetrain.
     * The joysticks' values map to percentage of the maximum velocities.
     * The velocities may be specified from either the robot's or field's frame of
     * reference.
     * Robot-centric: +x is forward, +y is left, +theta is CCW
     * Field-centric: origin is down-right, 0deg is up, +x is forward, +y is left,
     * +theta is CCW
     * direction.
     *      ___________
     *      |    |    | ^
     * (0,0).____|____| y, x-> 0->
     */
    if (drivetrainSubsystem != null) {
      // drivetrainSubsystem.setDefaultCommand(
      // new TeleopSwerve(
      // drivetrainSubsystem, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

      drivetrainSubsystem.setDefaultCommand(new TeleopSwervePlus(this, oi));
    }

    configureButtonBindings();
  }

  private void configureDefaultCommands() {
    if (holderSubsystem != null) {
      // holderSubsystem.setDefaultCommand(new DefaultHolderCommand(holderSubsystem));
    }

    if (indexerSubsystem != null) {
      // indexerSubsystem.setDefaultCommand(new
      // DefaultIndexerCommand(indexerSubsystem));
    }

    if (extenderSubsystem != null) {
      extenderSubsystem.setDefaultCommand(new DefaultExtenderCommand(extenderSubsystem));
    }

    if (stateMachineSubsystem != null) {
      // stateMachineSubsystem.setDefaultCommand(new DefaultStateMachineCommand());
    }

    if (rgbStatusSubsytem != null) {
      // rgbStatusSubsytem.setDefaultCommand(new DefaultRgbStatusCommand());
    }

    if (limelightObjectDetectionSubsystem != null) {
      limelightObjectDetectionSubsystem.setDefaultCommand(
          new DefaultLimelightObjectDectionCommand(limelightObjectDetectionSubsystem));
    }

    if (limelightScoringSubSystem != null) {
      limelightScoringSubSystem.setDefaultCommand(
          new DefaultLimelightScoringDectionCommand(limelightScoringSubSystem));
    }
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle
    oi.getFieldRelativeButton()
        .onTrue(
            Commands.either(
                Commands.runOnce(drivetrainSubsystem::disableFieldRelative, drivetrainSubsystem),
                Commands.runOnce(drivetrainSubsystem::enableFieldRelative, drivetrainSubsystem),
                drivetrainSubsystem::getFieldRelative));

    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrainSubsystem::zeroGyroscope));

    // x-stance
    oi.getXStanceButton()
        .onTrue(
            Commands.runOnce(
                () -> {
                  UseAutoAlign = true;
                  // robotTranslationMode = RobotTranslationMode.SCORE_PIECE;
                  // robotRotationMode = RobotRotationMode.SCORE_PIECE;
                }));
    oi.getXStanceButton()
        .onFalse(
            Commands.runOnce(
                () -> {
                  UseAutoAlign = false;
                  robotTranslationMode = RobotTranslationMode.DRIVER;
                  robotRotationMode = RobotRotationMode.DRIVER;
                }));

    // Raise/Lower Indexer Arm
    /*
    oi.getIndexerRotateUpButton()
        .onTrue(Commands.runOnce(indexerSubsystem::rotateUp, indexerSubsystem));
    oi.getIndexerRotateUpButton()
        .onFalse(Commands.runOnce(indexerSubsystem::rotateOff, indexerSubsystem));
    oi.getIndexerRotateDownButton()
        .onTrue(Commands.runOnce(indexerSubsystem::rotateDown, indexerSubsystem));
    oi.getIndexerRotateDownButton()
        .onFalse(Commands.runOnce(indexerSubsystem::rotateOff, indexerSubsystem));
                */
    // Indexer Intake/Eject
    /* oi.getIndexerIntakeButton()
        .onTrue(Commands.runOnce(indexerSubsystem::grabberIntake, indexerSubsystem));
    oi.getIndexerIntakeButton()
        .onFalse(Commands.runOnce(indexerSubsystem::grabberOff, indexerSubsystem));
    */
    /*oi.getIndexerEjectButton()
        .onTrue(Commands.runOnce(indexerSubsystem::grabberEject, indexerSubsystem));
    oi.getIndexerEjectButton()
        .onFalse(Commands.runOnce(indexerSubsystem::grabberOff, indexerSubsystem));*/

    // Indexer Open/Close
    // oi.getIndexerToggleOpenButton()
    //    .onTrue(Commands.runOnce(indexerSubsystem::toggleOpenClose, indexerSubsystem));

    // Holder Open
    // oi.getHodlerOpenButton().onTrue(Commands.runOnce(holderSubsystem::open, holderSubsystem));
    // oi.getHodlerOpenButton().onFalse(Commands.runOnce(holderSubsystem::close, holderSubsystem));

    /*oi.getIndexerEjectButton()
    .onTrue(
        new InstantCommand(
            () -> {
              if (this.robotStateMachine.getCurrentState().equals("readyToIntake")) {
                CommandScheduler.getInstance().schedule(new ExtenderReseatCommmand(this));
              }
            }));*/
    // oi.getIndexerEjectButton()
    //    .onFalse(Commands.runOnce(indexerSubsystem::grabberOff, indexerSubsystem));

    oi.getHodlerOpenButton()
        .onTrue(
            new InstantCommand(
                () -> {
                  boolean isRotationInTolerance =
                      Math.abs(
                              drivetrainSubsystem
                                  .getPose()
                                  .getRotation()
                                  .minus(Rotation2d.fromDegrees(180))
                                  .getDegrees())
                          < Constants.SCORING_ROTATION_TOLERANCE;
                  if (isRotationInTolerance
                      && robotStateMachine.getCurrentState().equals("carrying")) {
                    robotRotationMode = RobotRotationMode.SCORE_PIECE;
                  }
                }));
    /*
        oi.getAdjustElevatorUpButton()
            .onTrue(Commands.runOnce(elevatorSubsystem::goUp, elevatorSubsystem));
        oi.getAdjustElevatorUpButton()
            .onFalse(Commands.runOnce(elevatorSubsystem::off, elevatorSubsystem));

        oi.getAdjustElevatorDownButton()
            .onTrue(Commands.runOnce(elevatorSubsystem::goDown, elevatorSubsystem));
        oi.getAdjustElevatorDownButton()
            .onFalse(Commands.runOnce(elevatorSubsystem::off, elevatorSubsystem));
    */
    // Intake
    oi.getIntakeButton()
        .onTrue(Commands.runOnce(() -> robotStateMachine.fireEvent(new IntakePressed())));
    oi.getIntakeButton()
        .onFalse(Commands.runOnce(() -> robotStateMachine.fireEvent(new IntakeReleased())));

    oi.getHumanPlayerIntakeButton().onTrue(Commands.runOnce(() -> collectAtHumanPlayer = true));
    oi.getHumanPlayerIntakeButton().onFalse(Commands.runOnce(() -> collectAtHumanPlayer = false));

    // Scoring
    oi.getScoreButton()
        .onTrue(
            Commands.runOnce(
                () -> {
                  UseAutoAlign = oi.getXStanceButton().getAsBoolean();
                  if (UseAutoAlign && areWeAbleToScore() && scoringHeight != ScoringHeight.LOW) {
                    CommandScheduler.getInstance()
                        .schedule(
                            new AutoAlignToScore(
                                this, robotStateMachine, limelightScoringSubSystem));
                  }
                  // robotTranslationMode = RobotTranslationMode.SCORE_PIECE;
                  // robotRotationMode = RobotRotationMode.SCORE_PIECE;
                  else if (!UseAutoAlign
                      || scoringHeight == ScoringHeight.LOW
                      || pieceMode == PieceMode.CUBE) {

                    robotStateMachine.fireEvent(new ScorePressed());
                  }
                  // }
                }));

    // oi.getDeployHolderButton()
    // .onTrue(Commands.runOnce(() -> robotStateMachine.fireEvent(new
    // FinishScorePressed())));

    // extender buttons
    // oi.getExtenderStablizerButton()
    //     .onTrue(Commands.runOnce(extenderSubsystem::engageStablizer, extenderSubsystem));
    // oi.getExtenderStablizerDisengageButton()
    //     .onTrue(Commands.runOnce(extenderSubsystem::disengageStablizer, extenderSubsystem));

    // Scoring Height buttons
    oi.getLowGoalButton()
        .onTrue(
            Commands.runOnce(
                () -> {
                  ScoringHeight prev = scoringHeight;
                  scoringHeight = ScoringHeight.LOW;
                  if (scoringHeight != prev) {
                    robotStateMachine.fireEvent(new SwitchToLow());
                  }
                }));

    oi.getMiddleGoalButton()
        .onTrue(
            Commands.runOnce(
                () -> {
                  ScoringHeight prev = scoringHeight;
                  limelightScoringSubSystem.setScoringMode(ScoringMode.ReflectiveTapeMid);
                  scoringHeight = ScoringHeight.MEDIUM;
                  if (scoringHeight != prev) {
                    robotStateMachine.fireEvent(new SwitchToMidHigh());
                  }
                }));

    oi.getHighGoalButton()
        .onTrue(
            Commands.runOnce(
                () -> {
                  ScoringHeight prev = scoringHeight;
                  scoringHeight = ScoringHeight.HIGH;
                  limelightScoringSubSystem.setScoringMode(ScoringMode.ReflectiveTapeHigh);
                  if (scoringHeight != prev) {
                    robotStateMachine.fireEvent(new SwitchToMidHigh());
                  }
                }));

    // Game Piece Mode Buttons
    oi.getConeModeButton().onTrue(Commands.runOnce(() -> pieceMode = PieceMode.CONE));

    oi.getCubeModeButton().onTrue(Commands.runOnce(() -> pieceMode = PieceMode.CUBE));

    // oi.getVisionAssistButton().onTrue(Commands.runOnce(() -> visionEnabled =
    // true));
    // oi.getVisionAssistButton().onFalse(Commands.runOnce(() -> visionEnabled =
    // false));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    Command autoTest =
        Commands.sequence(
            Commands.runOnce(drivetrainSubsystem::enableXstance, drivetrainSubsystem),
            Commands.waitSeconds(5.0),
            Commands.runOnce(drivetrainSubsystem::disableXstance, drivetrainSubsystem));

    // add commands to the auto chooser
    autoChooser.addOption("Do Nothing", new InstantCommand());

    // demonstration of PathPlanner path group with event markers
    // autoChooser.addOption("Test Path", autoTest);

    // "auto" command for tuning the drive velocity PID
    // autoChooser.addOption(
    // "Drive Velocity Tuning",
    // Commands.sequence(
    // Commands.runOnce(drivetrainSubsystem::disableFieldRelative,
    // drivetrainSubsystem),
    // Commands.deadline(
    // Commands.waitSeconds(5.0),
    // Commands.run(
    // () -> drivetrainSubsystem.drive(1.5, 0.0, 0.0), drivetrainSubsystem))));

    // "auto" command for characterizing the drivetrain
    // autoChooser.addOption(
    // "Drive Characterization",
    // new FeedForwardCharacterization(
    // drivetrainSubsystem,
    // true,
    // new FeedForwardCharacterizationData("drive"),
    // drivetrainSubsystem::runCharacterizationVolts,
    // drivetrainSubsystem::getCharacterizationVelocity));

    autoChooser.addOption(
        "Place High",
        Commands.sequence(
            new InitializeRobotCommand(this), CommandFactory.getScoreWithHolderCommand(this)));

    autoChooser.addOption(
        "Place High, Taxi",
        Commands.sequence(
            new InitializeRobotCommand(this),
            CommandFactory.getScoreWithHolderCommand(this).withTimeout(6.5),
            new DriveDistance(drivetrainSubsystem, DriveDistance.Direction.BACKWARD, 3)));

    autoChooser.addOption(
        "Blue Place High, Balance",
        Commands.sequence(
            new InitializeRobotCommand(this),
            CommandFactory.getScoreWithHolderCommand(this).withTimeout(6.5),
            new DriveDistance(drivetrainSubsystem, DriveDistance.Direction.BACKWARD, 1.425, 0.3),
            new InstantCommand(() -> drivetrainSubsystem.setXStance(), drivetrainSubsystem)));
    // new AutoBalance(adis16470Gyro, drivetrainSubsystem)));

    autoChooser.addOption(
        "Red Place High, Balance",
        Commands.sequence(
            new InitializeRobotCommand(this),
            CommandFactory.getScoreWithHolderCommand(this).withTimeout(6.5),
            new DriveDistance(drivetrainSubsystem, DriveDistance.Direction.BACKWARD, 1.425, 0.3),
            new InstantCommand(() -> drivetrainSubsystem.setXStance(), drivetrainSubsystem)));

    autoChooser.addOption(
        "Place High, Taxi, Balance",
        Commands.sequence(
            new InitializeRobotCommand(this),
            CommandFactory.getScoreWithHolderCommand(this).withTimeout(6.5),
            new DriveDistance(
                drivetrainSubsystem, DriveDistance.Direction.BACKWARD, 1.4, 0.3, false),
            new DriveDistance(drivetrainSubsystem, DriveDistance.Direction.BACKWARD, 1.3, 0.2),
            new DriveDistance(
                drivetrainSubsystem, DriveDistance.Direction.FORWARD, 1.5 + 5 * .0254, 0.3),
            new InstantCommand(() -> drivetrainSubsystem.setXStance(), drivetrainSubsystem)));

    autoChooser.addOption(
        "Blue Place High, Taxi, AutoBalance",
        Commands.sequence(
            new InitializeRobotCommand(this),
            CommandFactory.getScoreWithHolderCommand(this).withTimeout(6.5),
            new DriveDistance(
                drivetrainSubsystem, DriveDistance.Direction.BACKWARD, 1.4, 0.3, false),
            new DriveDistance(drivetrainSubsystem, DriveDistance.Direction.BACKWARD, 1.3, 0.2),
            Commands.sequence(
                new DriveDistance(
                    drivetrainSubsystem, DriveDistance.Direction.FORWARD, 1.5 + 5 * .0254, 0.3),
                new InstantCommand(() -> drivetrainSubsystem.setXStance(), drivetrainSubsystem)),
            new WaitCommand(2),
            new AutoBalance(adis16470Gyro, drivetrainSubsystem)));

    autoChooser.addOption(
        "Red Place High, Taxi, AutoBalance",
        Commands.sequence(
            new InitializeRobotCommand(this),
            CommandFactory.getScoreWithHolderCommand(this).withTimeout(6.5),
            new DriveDistance(
                drivetrainSubsystem, DriveDistance.Direction.BACKWARD, 1.4, 0.3, false),
            new DriveDistance(drivetrainSubsystem, DriveDistance.Direction.BACKWARD, 1.3, 0.2),
            Commands.sequence(
                new DriveDistance(
                    drivetrainSubsystem, DriveDistance.Direction.FORWARD, 1.5 + 5 * .0254, 0.3),
                new InstantCommand(() -> drivetrainSubsystem.setXStance(), drivetrainSubsystem)),
            new WaitCommand(2),
            new AutoBalance(adis16470Gyro, drivetrainSubsystem)));
    // autoChooser.addOption(
    // "Auto Balance Test",
    // Commands.sequence(
    // // new InitializeRobotCommand(this, pieceMode, scoringHeight, new
    // Rotation2d(Math.PI)),
    // // CommandFactory.getScoreWithHolderCommand(this).raceWith(new
    // CustomWaitCommand(6.5)),
    // // new DriveDistance(drivetrainSubsystem, DriveDistance.Direction.BACKWARD,
    // 3, 0.4),
    // // new DriveDistance(drivetrainSubsystem, DriveDistance.Direction.FORWARD,
    // 1.5, 0.4),
    // new AutoBalance(adis16470Gyro, drivetrainSubsystem)));

    // autoChooser.addOption(
    // "Two Piece, Balance",
    // Commands.sequence(
    // new InitializeRobotCommand(this),
    // CommandFactory.getScoreWithHolderCommand(this)));

    autoChooser.addOption(
        "Blue, Two Piece, Taxi",
        Commands.sequence(
            new InitializeRobotCommand(this, Constants.CONE_NODE_1),
            new InstantCommand(
                () ->
                    CommandScheduler.getInstance()
                        .schedule(CommandFactory.getScoreWithHolderCommand(this))),
            new WaitCommand(1.2),
            new SwerveToWaypointCommand(
                drivetrainSubsystem, Constants.NEUTRAL_PIECE_1, Constants.FLAT_LANE_OUT_WAYPOINTS),
            new InstantCommand(() -> pieceMode = PieceMode.CUBE),
            Commands.race(
                new TeleopSwervePlus(this, oi),
                Commands.sequence(
                    new InstantCommand(() -> robotStateMachine.fireEvent(new IntakePressed())),
                    new InstantCommand(
                        () -> robotTranslationMode = RobotTranslationMode.AUTO_PIECE_TRACKING),
                    new WaitUntilCommand(
                            () -> "carrying".equals(robotStateMachine.getCurrentState()))
                        .withTimeout(3),
                    new InstantCommand(() -> robotStateMachine.fireEvent(new IntakeReleased())),
                    new InstantCommand(() -> robotTranslationMode = RobotTranslationMode.DRIVER))),
            new SwerveToWaypointCommand(
                drivetrainSubsystem, Constants.CUBE_NODE_1, Constants.FLAT_LANE_IN_WAYPOINTS),
            Commands.race(
                new TeleopSwervePlus(this, oi),
                Commands.sequence(
                    new InstantCommand(() -> robotStateMachine.fireEvent(new ScorePressed())),
                    new WaitCommand(2.55)))));
    // new SwerveToWaypointCommand(
    // drivetrainSubsystem,
    // Constants.NEUTRAL_PIECE_1,
    // Constants.FLAT_LANE_OUT_WAYPOINTS)));
    autoChooser.addOption(
        "Red, Two Piece, Taxi",
        Commands.sequence(
            new InitializeRobotCommand(this, Constants.RED_CONE_NODE_1),
            new InstantCommand(
                () ->
                    CommandScheduler.getInstance()
                        .schedule(CommandFactory.getScoreWithHolderCommand(this))),
            new WaitCommand(1.2),
            new SwerveToWaypointCommand(
                drivetrainSubsystem, Constants.RED_NEUTRAL_PIECE_1, Constants.RED_FLAT_LANE_OUT_WAYPOINTS),
            new InstantCommand(() -> pieceMode = PieceMode.CUBE),
            Commands.race(
                new TeleopSwervePlus(this, oi),
                Commands.sequence(
                    new InstantCommand(() -> robotStateMachine.fireEvent(new IntakePressed())),
                    new InstantCommand(
                        () -> robotTranslationMode = RobotTranslationMode.AUTO_PIECE_TRACKING),
                    new WaitUntilCommand(
                            () -> "carrying".equals(robotStateMachine.getCurrentState()))
                        .withTimeout(3),
                    new InstantCommand(() -> robotStateMachine.fireEvent(new IntakeReleased())),
                    new InstantCommand(() -> robotTranslationMode = RobotTranslationMode.DRIVER))),
            new SwerveToWaypointCommand(
                drivetrainSubsystem, Constants.CUBE_NODE_1, Constants.FLAT_LANE_IN_WAYPOINTS),
            Commands.race(
                new TeleopSwervePlus(this, oi),
                Commands.sequence(
                    new InstantCommand(() -> robotStateMachine.fireEvent(new ScorePressed())),
                    new WaitCommand(2.55)))));

    // autoChooser.addOption(
    // "Bump-side Two Piece, Taxi",
    // Commands.sequence(
    // new InitializeRobotCommand(this, Constants.CONE_NODE_1),
    // new InstantCommand(
    // () ->
    // CommandScheduler.getInstance()
    // .schedule(CommandFactory.getScoreWithHolderCommand(this))),
    // new WaitCommand(3),
    // new InstantCommand(() -> pieceMode = PieceMode.CUBE),
    // new SwerveToWaypointCommand(drivetrainSubsystem,
    // Constants.CONE_NODE_1plus)));

    // autoChooser.addOption(
    //     "Three Piece Bump",
    //     Commands.sequence(
    //         // Start
    //         new InitializeRobotCommand(this, Constants.BUMP_START),
    //         // Intake to launch cube
    //         new InstantCommand(() -> scoringHeight = ScoringHeight.LOW),
    //         new InstantCommand(() -> robotStateMachine.fireEvent(new IntakePressed())),
    //         new WaitCommand(0.5),
    //         // Intake the first cone
    //         new SwerveToWaypointCommand(
    //             drivetrainSubsystem, Constants.LAYING_DOWN_4, Constants.LAYING_DOWN_4_WAYPOINTS),
    //         // Drop off the cone
    //         new SwerveToWaypointCommand(drivetrainSubsystem, Constants.CONE_DROPOFF),
    //         new InstantCommand(() -> robotStateMachine.fireEvent(new IntakeReleased())),
    //         new InstantCommand(() -> robotStateMachine.fireEvent(new ScorePressed())),
    //         // Drive to next cone
    //         new SwerveToWaypointCommand(
    //             drivetrainSubsystem,
    //             Constants.LAYING_DOWN_3_APPROACH_2,
    //             Constants.LAYING_DOWN_3_WAYPOINTS),
    //         new InstantCommand(() -> scoringHeight = ScoringHeight.HIGH),
    //         new InstantCommand(() -> robotStateMachine.fireEvent(new IntakePressed())),
    //         // Intake cone while moving back
    //         new SwerveToWaypointCommand(
    //             drivetrainSubsystem, Constants.CONE_PICKUP, Constants.PICKUP_WAYPOINTS),
    //         new InstantCommand(() -> robotStateMachine.fireEvent(new IntakeReleased())),
    //         // Go to scoring location
    //         new SwerveToWaypointCommand(
    //             drivetrainSubsystem, Constants.CONE_PLACEMENT_5,
    // Constants.PLACEMENT_5_WAYPOINTS),
    //         // Score Cone High
    //         Commands.race(
    //             new TeleopSwervePlus(this, oi),
    //             Commands.sequence(
    //                 new AutoAlignToScore(this, robotStateMachine, limelightScoringSubSystem),
    //                 new WaitUntilCommand(
    //                         () -> "readyToIntake".equals(robotStateMachine.getCurrentState()))
    //                     .withTimeout(3))),
    //         // Transfer Cone and move to next scoring location
    //         new SwerveToWaypointCommand(
    //             drivetrainSubsystem, Constants.CONE_PLACEMENT_6,
    // Constants.PLACEMENT_5_WAYPOINTS),
    //         // Score Cone High
    //         Commands.race(
    //             new TeleopSwervePlus(this, oi),
    //             Commands.sequence(
    //                 new AutoAlignToScore(
    //                     robotContainer, robotStateMachine, limelightScoringSubSystem),
    //                 new WaitUntilCommand(
    //                         () -> "readyToIntake".equals(robotStateMachine.getCurrentState()))
    //                     .withTimeout(3)))));

    autoChooser.addDefaultOption(
        "Two Plus One Piece Bump",
        Commands.sequence(
            // Start
            new InitializeRobotCommand(this, Constants.BUMP_START),
            // Intake to launch cube
            new InstantCommand(() -> robotStateMachine.fireEvent(new IntakePressed())),
            new WaitCommand(0.5),
            // Intake the first cone
            new SwerveToWaypointCommand(
                drivetrainSubsystem, Constants.LAYING_DOWN_4, Constants.LAYING_DOWN_4_WAYPOINTS),
            // Drive to cone node 6
            new SwerveToWaypointCommand(
                    drivetrainSubsystem,
                    Constants.CONE_PLACEMENT_6,
                    Constants.BUMP_CENTER_WAYPOINTS)
                .raceWith(
                    new WaitUntilCommand(
                        () ->
                            Constants.CONE_PLACEMENT_6
                                    .getTranslation()
                                    .getDistance(drivetrainSubsystem.getPose().getTranslation())
                                < 0.25)),
            new InstantCommand(() -> robotStateMachine.fireEvent(new IntakeReleased())),
            new WaitUntilCommand(() -> "carrying".equals(robotStateMachine.getCurrentState())),
            // Score Cone High
            Commands.race(
                new TeleopSwervePlus(this, oi),
                Commands.sequence(
                    new AutoAlignToScore(this, this.robotStateMachine, limelightScoringSubSystem),
                    new WaitUntilCommand(
                            () -> "readyToIntake".equals(robotStateMachine.getCurrentState()))
                        .withTimeout(3))),
            // Drive to next cone
            new InstantCommand(
                () ->
                    drivetrainSubsystem.resetOdometry(
                        CommandFactory.getAllianceCorrectedPose(Constants.SCORED_NODE_6))),
            new WaitCommand(1.5),
            new InstantCommand(
                () -> {
                  scoringHeight = ScoringHeight.LOW;
                  pieceMode = PieceMode.CONE;
                }),
            Commands.parallel(
                new SwerveToWaypointCommand(
                    drivetrainSubsystem,
                    Constants.FINAL_CONE_GRAB,
                    Constants.REVERSE_BUMP_CENTER_WAYPOINTS),
                Commands.sequence(
                    new WaitCommand(1.6),
                    new InstantCommand(() -> robotStateMachine.fireEvent(new IntakePressed())))),
            Commands.race(
                new SwerveToWaypointCommand(
                    drivetrainSubsystem,
                    Constants.BUMP_CENTER_LANE_CLOSE,
                    Constants.BUMP_CENTER_WAYPOINTS_2),
                Commands.sequence(
                    new WaitCommand(1.7),
                    new InstantCommand(() -> robotStateMachine.fireEvent(new ScorePressed()))),
                new WaitCommand(.2))));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  private void defineSubsystems() {
    if (Constants.HARDWARE_CONFIG_HAS_DRIVETRAIN) {
      drivetrainSubsystem = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
    }

    if (Constants.HARDWARE_CONFIG_HAS_ELEVATOR) {
      elevatorSubsystem = new ElevatorSubsystem();
    }

    if (Constants.HARDWARE_CONFIG_HAS_EXTENDER) {
      extenderSubsystem = new ExtenderSubsystem();
    }

    if (Constants.HARDWARE_CONFIG_HAS_HOLDER) {
      holderSubsystem = new HolderSubsystem();
    }

    if (Constants.HARDWARE_CONFIG_HAS_INDEXER) {
      indexerSubsystem = new IndexerSubsystem();
    }

    if (Constants.HARDWARE_CONFIG_HAS_RGB) {
      rgbStatusSubsytem = new RGBStatusSubsytem(this);
    }

    if (Constants.HARDWARE_CONFIG_HAS_BOTH_LIMELIGHTS) {
      LimelightHelpers.profileJSON = true;
      LimelightHelpers.getObjectMapper();

      limelightObjectDetectionSubsystem = new LimelightObjectDetection(this);
      limelightScoringSubSystem = new LimelightScoring();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public boolean isVisionOn() {
    return oi.getVisionAssistButton().getAsBoolean();
  }

  public boolean inCarryState() {
    return robotStateMachine.getCurrentState().equals("carrying");
  }

  public boolean areWeAbleToScore() {
    boolean isTyInTolerance = limelightScoringSubSystem.isWithinTolerance();
    boolean isDistanceInTolerance = limelightScoringSubSystem.isWithinDistanceTolerance();
    boolean isRotationInTolerance =
        Math.abs(
                drivetrainSubsystem
                    .getPose()
                    .getRotation()
                    .minus(Rotation2d.fromDegrees(180))
                    .getDegrees())
            < Constants.SCORING_ROTATION_TOLERANCE;

    return robotStateMachine.getCurrentState().equals("carrying")
        && isTyInTolerance
        && isDistanceInTolerance
        && isRotationInTolerance;
  }
}
