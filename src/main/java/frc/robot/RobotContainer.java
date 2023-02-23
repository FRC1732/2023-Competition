// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

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
import frc.robot.commands.DefaultCommands.DefaultExtenderCommand;
import frc.robot.commands.DefaultCommands.DefaultHolderCommand;
import frc.robot.commands.DefaultCommands.DefaultIndexerCommand;
import frc.robot.commands.DefaultCommands.DefaultStateMachineCommand;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.TeleopSwerve;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.HolderSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
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

  public Drivetrain drivetrain;
  public IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  public HolderSubsystem holderSubsystem = new HolderSubsystem();
  public ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();
  public StateMachineSubsystem stateMachineSubsystem = new StateMachineSubsystem();
  public ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

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
  private RobotContainer() {

    configureDriveTrain();

    // disable all telemetry in the LiveWindow to reduce the processing during each
    // iteration
    LiveWindow.disableAllTelemetry();

    updateOI();
    configureDefaultCommands();
    configureButtonBindings();
    configureAutoCommands();
  }

  private void configureDriveTrain() {
    GyroIO gyro = new GyroIoADIS16470();

    SwerveModule flModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                0,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER,
                FRONT_LEFT_MODULE_STEER_OFFSET),
            0,
            MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule frModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                1,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER,
                FRONT_RIGHT_MODULE_STEER_OFFSET),
            1,
            MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule blModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                2,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER,
                BACK_LEFT_MODULE_STEER_OFFSET),
            2,
            MAX_VELOCITY_METERS_PER_SECOND);

    SwerveModule brModule =
        new SwerveModule(
            new SwerveModuleIOTalonFX(
                3,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER,
                BACK_RIGHT_MODULE_STEER_OFFSET),
            3,
            MAX_VELOCITY_METERS_PER_SECOND);

    drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
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

    /*
     * Set up the default command for the drivetrain.
     * The joysticks' values map to percentage of the maximum velocities.
     * The velocities may be specified from either the robot's or field's frame of reference.
     * Robot-centric: +x is forward, +y is left, +theta is CCW
     * Field-centric: origin is down-right, 0deg is up, +x is forward, +y is left, +theta is CCW
     * direction.
     *      ___________
     *      |    |    | ^
     * (0,0).____|____| y, x-> 0->
     */
    if (drivetrain != null) {
      drivetrain.setDefaultCommand(
          new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate));
    }
  }

  private void configureDefaultCommands() {
    if (holderSubsystem != null) {
      holderSubsystem.setDefaultCommand(new DefaultHolderCommand(holderSubsystem));
    }

    if (indexerSubsystem != null) {
      indexerSubsystem.setDefaultCommand(new DefaultIndexerCommand(indexerSubsystem));
    }

    if (extenderSubsystem != null) {
      extenderSubsystem.setDefaultCommand(new DefaultExtenderCommand(extenderSubsystem));
    }

    if (stateMachineSubsystem != null) {
      stateMachineSubsystem.setDefaultCommand(new DefaultStateMachineCommand());
    }
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle
    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    // Intake buttons
    // oi.getIntakeButton().onTrue(Commands.runOnce(intakeSubsystem::on, intakeSubsystem));
    // oi.getIntakeButton().onFalse(Commands.runOnce(intakeSubsystem::off, intakeSubsystem));

    // Indexer buttons
    oi.getGrabberConeButton()
        .onTrue(
            Commands.runOnce(indexerSubsystem::open, indexerSubsystem)
                .andThen(Commands.runOnce(indexerSubsystem::grabberEject, indexerSubsystem)));
    oi.getGrabberConeButton()
        .onFalse(Commands.runOnce(indexerSubsystem::grabberOff, indexerSubsystem));

    // oi.getGrabberCubeButton()
    //     .onTrue(
    //         Commands.runOnce(indexerSubsystem::close, indexerSubsystem)
    //             .andThen(Commands.runOnce(indexerSubsystem::grabberOn, indexerSubsystem))
    //             .andThen(Commands.runOnce(intakeSubsystem::on, intakeSubsystem)));
    // oi.getGrabberCubeButton()
    //     .onFalse(
    //         Commands.runOnce(indexerSubsystem::grabberOff, indexerSubsystem)
    //             .andThen(Commands.runOnce(intakeSubsystem::off, intakeSubsystem)));

    oi.getGrabberEjectButton().onTrue(Commands.runOnce(indexerSubsystem::eject, indexerSubsystem));
    oi.getGrabberEjectButton()
        .onFalse(Commands.runOnce(indexerSubsystem::grabberOff, indexerSubsystem));

    oi.getIndexerRotateUpButton()
        .onTrue(Commands.runOnce(indexerSubsystem::rotateUp, indexerSubsystem));
    oi.getIndexerRotateUpButton()
        .onFalse(Commands.runOnce(indexerSubsystem::rotateOff, indexerSubsystem));

    oi.getIndexerRotateDownButton()
        .onTrue(Commands.runOnce(indexerSubsystem::rotateDown, indexerSubsystem));
    oi.getIndexerRotateDownButton()
        .onFalse(Commands.runOnce(indexerSubsystem::rotateOff, indexerSubsystem));

    oi.getIndexerOpenButton().onTrue(Commands.runOnce(indexerSubsystem::open, indexerSubsystem));

    oi.getIndexerCloseButton().onTrue(Commands.runOnce(indexerSubsystem::close, indexerSubsystem));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    Command autoTest =
        Commands.sequence(
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(5.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain));

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // demonstration of PathPlanner path group with event markers
    autoChooser.addOption("Test Path", autoTest);

    // "auto" command for tuning the drive velocity PID
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.deadline(
                Commands.waitSeconds(5.0),
                Commands.run(() -> drivetrain.drive(1.5, 0.0, 0.0), drivetrain))));

    // "auto" command for characterizing the drivetrain
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrain::runCharacterizationVolts,
            drivetrain::getCharacterizationVelocity));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
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
