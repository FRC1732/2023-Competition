// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.Map;

public class LimelightScoring extends SubsystemBase {
  // limelight hostname: http://limelight-scoring.local:5801
  private final String LIMELIGHTNAME = "limelight-scoring";
  private final Pose2d EMPTY_POSE2D = new Pose2d();

  private HttpCamera LLFeed;
  private MjpegServer server;

  private LimelightHelpers.LimelightResults llresults;
  private boolean havePosition = false;
  private Pose2d position = EMPTY_POSE2D;

  private NetworkTable table;
  private NetworkTableEntry tv;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry pipeline;

  private ScoringMode currentScoringMode = ScoringMode.Undefined;

  private double reflectiveTv;
  private double reflectiveTx;
  private double reflectiveTy;
  private double reflectiveTa;
  private int pipelineVal;

  /** Creates a new Limelight. */
  public LimelightScoring() {
    configureNetworkTableEntries();
    configureShuffleBoard();

    LimelightHelpers.getLatestResults(LIMELIGHTNAME);
  }

  private void configureNetworkTableEntries() {
    table = NetworkTableInstance.getDefault().getTable(LIMELIGHTNAME);
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    pipeline = table.getEntry("pipeline");
  }

  private void configureShuffleBoard() {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab(LIMELIGHTNAME);
    setScoringMode(ScoringMode.ReflectiveTapeHigh);
    LLFeed = new HttpCamera(LIMELIGHTNAME, "http://10.17.32.11:5800/stream.mjpg");
    server = CameraServer.addSwitchedCamera("Object Camera");
    server.setSource(LLFeed);
    LLFeed.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    tab.add(server.getSource())
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(5, 0)
        .withSize(5, 5)
        .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));

    tab.addNumber("tv - Valid Targets", () -> reflectiveTv);
    tab.addNumber("tx - Horiz Offset", () -> reflectiveTx);
    tab.addNumber("ty - Vert Offset", () -> reflectiveTy);
    tab.addNumber("ta - Target Area", () -> reflectiveTa);
    tab.addBoolean("Target Acquired", () -> reflectiveTv > 0);

    tab.addNumber("Tag Sighted", () -> getPriorityTag());
    tab.addString("Robot Pose", () -> getPose2d().toString());

    tab.addNumber("Selected Pipeline", () -> pipelineVal);

    // tab.addBoolean(
    //     "Elevator At Starting Position",
    //     () -> RobotContainer.getInstance().elevatorSubsystem.isAtStartPoint());
  }

  @Override
  public void periodic() {
    // read and store values periodically
    if (currentScoringMode == ScoringMode.AprilTag) {
      llresults = LimelightHelpers.getLatestResults(LIMELIGHTNAME);

      try {
        position = llresults.targetingResults.getBotPose2d();
      } catch (NullPointerException e) {
        position = EMPTY_POSE2D;
      }

      havePosition = EMPTY_POSE2D.equals(position);

      if (llresults != null) {
        // System.out.println(LimelightHelpers.getJSONDump(""));
      }
    } else {
      // ReflectiveTape or Undefined
      reflectiveTv = tv.getDouble(0);
      reflectiveTx = tx.getDouble(0);
      reflectiveTy = ty.getDouble(0);
      reflectiveTa = ta.getDouble(0);
    }
    pipelineVal = (int) pipeline.getDouble(-1);
  }

  /**
   * Provides result for if a reflective tape target has been sighted
   *
   * @return true if reflective tape target found
   */
  public boolean hasTarget() {
    return reflectiveTv > 0;
  }

  public int getPriorityTag() {
    if (havePosition) {
      return (int) llresults.targetingResults.targets_Fiducials[0].fiducialID;
    }

    return -1;
  }

  public boolean isAligned() {
    double tx = getTx();
    // Low goal is off by 1 degree
    if (getTy() < 0) {
      tx = tx - 1;
    }
    return hasTarget() && Math.abs(tx) < 1.75;
  }

  public boolean isWithinTolerance() {
    double tx = getTx();
    // Low goal is off by 1 degree
    if (getTy() < 0) {
      tx = tx - 1;
    }
    return hasTarget() && Math.abs(tx) < Constants.SCORING_TRANSLATION_TOLERANCE;
  }

  public double interpDistance() {
    double ty = getTy();
    if (currentScoringMode == ScoringMode.ReflectiveTapeHigh) {
      double x1 = 9.6;
      double y1 = 0;
      double x2 = 13.06;
      double y2 = 33.375;
      double temp = interp(x1, y1, x2, y2, ty);
      System.out.println(currentScoringMode + " INTERP DIST H" + ty + " " + temp);
      return temp;
    } else {
      double x1 = -18.15;
      double y1 = 0;
      double x2 = -3;
      double y2 = 33.375;
      double temp = interp(x1, y1, x2, y2, ty);
      System.out.println(currentScoringMode + "INTERP DIST M" + ty + " " + temp);
      return temp;
    }
  }

  public double interpXSetpoint() {
    double ty = getTy();
    if (currentScoringMode == ScoringMode.ReflectiveTapeHigh) {
      double x1 = 9.6;
      double y1 = 1.41;
      double x2 = 13.06;
      double y2 = .43;
      return interp(x1, y1, x2, y2, ty);
    } else {
      double x1 = -18.15;
      double y1 = 2.48;
      double x2 = -3;
      double y2 = .89;
      return interp(x1, y1, x2, y2, ty);
    }
  }

  private double interp(double x1, double y1, double x2, double y2, double lookup) {
    double slope = (y2 - y1) / (x2 - x1);
    double b = y1 - slope * x1;
    return lookup * slope + b;
  }

  public boolean isWithinDistanceTolerance() {
    return hasTarget() && position.getX() < Constants.SCORING_DISTANCE_TOLERANCE;
  }

  /**
   * Provides X (horizontal) value from limelight target when in Reflective Tape mode. Undefined
   * when in AprilTag mode.
   *
   * @return a horizontal "distance" from center of camera. Positive to the right.
   */
  public Double getTx() {
    return reflectiveTx;
  }

  /**
   * Provides Y (vertical) value from limelight target when in Reflective Tape mode. Undefined when
   * in AprilTag mode.
   *
   * @return a vertical "distance" from center of camera. Positive is up.
   */
  public Double getTy() {
    return reflectiveTy;
  }

  public Double getTa() {
    return reflectiveTa;
  }

  /**
   * Provide robot pose pased on April Tag inputs. Undefined when in Reflective Tape mode.
   *
   * @return the 2D pose (translation and rotation) of the robot based on April Tag sightings.
   */
  public Pose2d getPose2d() {
    return position;
  }

  /**
   * Sets the camera mode and pipeline for the type of visual tracking to be used.
   *
   * @param mode Provides the scoring mode
   */
  public void setScoringMode(ScoringMode mode) {
    currentScoringMode = mode;

    // set the pipeline to match the scoring mode.
    if (currentScoringMode == ScoringMode.AprilTag) {
      // FIXME: verify pipeline indices
      pipeline.setDouble(2);
    } else if (currentScoringMode == ScoringMode.ReflectiveTapeHigh) {
      pipeline.setDouble(0);

    } else if (currentScoringMode == ScoringMode.ReflectiveTapeMid) {
      pipeline.setDouble(1);
    }
  }

  public ScoringMode getScoringMode() {
    return currentScoringMode;
  }

  public enum ScoringMode {
    AprilTag,
    ReflectiveTapeMid,
    ReflectiveTapeHigh,
    Undefined;
  }
}
