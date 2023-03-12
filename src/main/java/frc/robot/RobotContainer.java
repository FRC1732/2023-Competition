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
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIoADIS16470;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.DefaultCommands.DefaultLimelightObjectDectionCommand;
import frc.robot.commands.DefaultCommands.DefaultLimelightScoringDectionCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
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
import frc.robot.subsystems.LimelightObjectDetection;
import frc.robot.subsystems.LimelightScoring;
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
    SLOW_MODE
  }

  public enum RobotRotationMode {
    DRIVER,
    PIECE_TRACKING,
    SCORE_PIECE
  }

  // public boolean visionEnabled = true;

  public PieceMode pieceMode;
  public ScoringHeight scoringHeight;
  public RobotTranslationMode robotTranslationMode;
  public RobotRotationMode robotRotationMode;

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

    updateOI();
    configureDefaultCommands();
    configureAutoCommands();
    robotStateMachine = new RobotStateMachine(this);
    stateMachineSubsystem = new StateMachineSubsystem(robotStateMachine);
  }

  private void configureDriveTrain() {
    gyro = new GyroIoADIS16470();

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
      // extenderSubsystem.setDefaultCommand(new
      // DefaultExtenderCommand(extenderSubsystem));
    }

    if (stateMachineSubsystem != null) {
      // stateMachineSubsystem.setDefaultCommand(new DefaultStateMachineCommand());
    }

    if (rgbStatusSubsytem != null) {
      // rgbStatusSubsytem.setDefaultCommand(new DefaultRgbStatusCommand());
    }

    if (limelightObjectDetectionSubsystem != null) {
      limelightObjectDetectionSubsystem.setDefaultCommand(
          new DefaultLimelightObjectDectionCommand());
    }

    if (limelightScoringSubSystem != null) {
      limelightScoringSubSystem.setDefaultCommand(new DefaultLimelightScoringDectionCommand());
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
        .onTrue(Commands.runOnce(drivetrainSubsystem::enableXstance, drivetrainSubsystem));
    oi.getXStanceButton()
        .onFalse(Commands.runOnce(drivetrainSubsystem::disableXstance, drivetrainSubsystem));

    // Raise/Lower Indexer Arm
    oi.getIndexerRotateUpButton()
        .onTrue(Commands.runOnce(indexerSubsystem::rotateUp, indexerSubsystem));
    oi.getIndexerRotateUpButton()
        .onFalse(Commands.runOnce(indexerSubsystem::rotateOff, indexerSubsystem));
    oi.getIndexerRotateDownButton()
        .onTrue(Commands.runOnce(indexerSubsystem::rotateDown, indexerSubsystem));
    oi.getIndexerRotateDownButton()
        .onFalse(Commands.runOnce(indexerSubsystem::rotateOff, indexerSubsystem));

    // Indexer Intake/Eject
    oi.getIndexerIntakeButton()
        .onTrue(Commands.runOnce(indexerSubsystem::grabberIntake, indexerSubsystem));
    oi.getIndexerIntakeButton()
        .onFalse(Commands.runOnce(indexerSubsystem::grabberOff, indexerSubsystem));
    oi.getIndexerEjectButton()
        .onTrue(Commands.runOnce(indexerSubsystem::grabberEject, indexerSubsystem));
    oi.getIndexerEjectButton()
        .onFalse(Commands.runOnce(indexerSubsystem::grabberOff, indexerSubsystem));

    // Indexer Open/Close
    oi.getIndexerToggleOpenButton()
        .onTrue(Commands.runOnce(indexerSubsystem::toggleOpenClose, indexerSubsystem));

    // Holder Open
    oi.getHodlerOpenButton().onTrue(Commands.runOnce(holderSubsystem::open, holderSubsystem));
    oi.getHodlerOpenButton().onFalse(Commands.runOnce(holderSubsystem::close, holderSubsystem));

    oi.getAdjustElevatorUpButton()
        .onTrue(Commands.runOnce(elevatorSubsystem::goUp, elevatorSubsystem));
    oi.getAdjustElevatorUpButton()
        .onFalse(Commands.runOnce(elevatorSubsystem::off, elevatorSubsystem));

    oi.getAdjustElevatorDownButton()
        .onTrue(Commands.runOnce(elevatorSubsystem::goDown, elevatorSubsystem));
    oi.getAdjustElevatorDownButton()
        .onFalse(Commands.runOnce(elevatorSubsystem::off, elevatorSubsystem));

    // Intake
    oi.getIntakeButton()
        .onTrue(Commands.runOnce(() -> robotStateMachine.fireEvent(new IntakePressed())));
    oi.getIntakeButton()
        .onFalse(Commands.runOnce(() -> robotStateMachine.fireEvent(new IntakeReleased())));

    // Scoring
    oi.getScoreButton()
        .onTrue(Commands.runOnce(() -> robotStateMachine.fireEvent(new ScorePressed())));

    // oi.getDeployHolderButton()
    // .onTrue(Commands.runOnce(() -> robotStateMachine.fireEvent(new
    // FinishScorePressed())));

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
                  if (scoringHeight != prev) {
                    robotStateMachine.fireEvent(new SwitchToMidHigh());
                  }
                }));

    // Game Piece Mode Buttons
    oi.getConeModeButton().onTrue(Commands.runOnce(() -> pieceMode = PieceMode.CONE));

    oi.getCubeModeButton().onTrue(Commands.runOnce(() -> pieceMode = PieceMode.CUBE));

    // oi.getVisionAssistButton().onTrue(Commands.runOnce(() -> visionEnabled = true));
    // oi.getVisionAssistButton().onFalse(Commands.runOnce(() -> visionEnabled = false));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    Command autoTest =
        Commands.sequence(
            Commands.runOnce(drivetrainSubsystem::enableXstance, drivetrainSubsystem),
            Commands.waitSeconds(5.0),
            Commands.runOnce(drivetrainSubsystem::disableXstance, drivetrainSubsystem));

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // demonstration of PathPlanner path group with event markers
    autoChooser.addOption("Test Path", autoTest);

    // "auto" command for tuning the drive velocity PID
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrainSubsystem::disableFieldRelative, drivetrainSubsystem),
            Commands.deadline(
                Commands.waitSeconds(5.0),
                Commands.run(
                    () -> drivetrainSubsystem.drive(1.5, 0.0, 0.0), drivetrainSubsystem))));

    // "auto" command for characterizing the drivetrain
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drivetrainSubsystem,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrainSubsystem::runCharacterizationVolts,
            drivetrainSubsystem::getCharacterizationVelocity));

    autoChooser.addOption(
        "Place High Cone",
        Commands.sequence(
            new InitializeRobotCommand(this, pieceMode, scoringHeight, new Rotation2d(Math.PI)),
            CommandFactory.getScoreWithHolderCommand(this)));

    autoChooser.addOption(
        "Place High Cone and Drive Back",
        Commands.sequence(
            // new InitializeRobotCommand(this, pieceMode, scoringHeight, new Rotation2d(Math.PI)),
            // CommandFactory.getScoreWithHolderCommand(this).raceWith(new CustomWaitCommand(6.5)),
            new DriveDistance(drivetrainSubsystem, 1)));

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
      limelightObjectDetectionSubsystem = new LimelightObjectDetection();
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
}
