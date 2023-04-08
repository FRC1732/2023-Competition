// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.PieceMode;
import frc.robot.subsystems.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.LimelightHelpers.LimelightTarget_Detector;
import java.util.Map;

public class LimelightObjectDetection extends SubsystemBase {
  // limelight hostname: http://limelight-objectd.local:5801
  private final String LIMELIGHTNAME = "limelight-objectd";
  private final String CONE_LABEL = "cone";
  private final String CUBE_LABEL = "cube";

  private HttpCamera LLFeed;
  private MjpegServer server;

  private LimelightHelpers.LimelightResults llresults;
  private boolean detectionOn = false;

  private boolean coneTarget = false;
  private boolean cubeTarget = false;
  private Translation2d conePose2d = null;
  private Translation2d cubePose2d = null;
  private double coneConfidence = 0.0;
  private double cubeConfidence = 0.0;

  private NetworkTable table;
  private NetworkTableEntry tv;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry pipeline;

  private PieceMode currentPieceMode;
  private RobotContainer robotContainer;

  private double limelightTv;
  private double limelightTx;
  private double limelightTy;
  private double limelightTa;
  private int pipelineVal;

  private void configureNetworkTableEntries() {
    table = NetworkTableInstance.getDefault().getTable(LIMELIGHTNAME);
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    pipeline = table.getEntry("pipeline");
  }

  /** Creates a new Limelight. */
  public LimelightObjectDetection(RobotContainer robotContainer) {
    configureNetworkTableEntries();
    configureShuffleBoard();
    LimelightHelpers.getFirstParse();
    this.robotContainer = robotContainer;
  }

  private void configureShuffleBoard() {
    // if (Constants.DEBUGGING) {
    ShuffleboardTab tab;
    tab = Shuffleboard.getTab(LIMELIGHTNAME);

    LLFeed = new HttpCamera(LIMELIGHTNAME, "http://10.17.32.12:5800/stream.mjpg");
    server = CameraServer.addSwitchedCamera("Object Camera");
    server.setSource(LLFeed);
    LLFeed.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    tab.add(server.getSource())
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(5, 0)
        .withSize(5, 5)
        .withProperties(Map.of("Show Crosshair", false, "Show Controls", false));

    tab.addBoolean("Cube Detected", () -> cubeTarget);
    tab.addBoolean("Cone Detected", () -> coneTarget);

    tab.addDouble("Cone X", () -> conePose2d != null ? conePose2d.getX() : 0);
    tab.addDouble("Cone Y", () -> conePose2d != null ? conePose2d.getY() : 0);
    tab.addDouble("Cube X", () -> cubePose2d != null ? cubePose2d.getX() : 0);
    tab.addDouble("Cube Y", () -> cubePose2d != null ? cubePose2d.getY() : 0);

    tab.addDouble("Cone Confidence", () -> coneConfidence);
    tab.addDouble("Cube Confidence", () -> cubeConfidence);
    // } else {
    // competition shuffleboard
    // what do we want here?
    // }
  }

  @Override
  public void periodic() {
    // read and store values periodically
    if (detectionOn) {
      if (currentPieceMode != robotContainer.pieceMode) {
        setPieceMode(robotContainer.pieceMode);
      }

      // note: because parsing the JSON method takes ~2.5ms, only do it when needed.
      // llresults = LimelightHelpers.getLatestResults(LIMELIGHTNAME);
      // processLlResults(llresults);
      limelightTv = tv.getDouble(0);
      limelightTx = tx.getDouble(0);
      limelightTy = ty.getDouble(0);
      limelightTa = ta.getDouble(0);
      boolean found = false;
      if (limelightTv > 0) {
        found = true;
      } else {
        coneTarget = false;
        cubeTarget = false;
      }

      if (currentPieceMode == PieceMode.CONE && found) {
        conePose2d = new Translation2d(limelightTx, limelightTy);
        coneTarget = true;
      }
      if (currentPieceMode == PieceMode.CUBE && found) {
        cubePose2d = new Translation2d(limelightTx, limelightTy);
        cubeTarget = true;
      }

      if (llresults != null) {
        // System.out.println(LimelightHelpers.getJSONDump(LIMELIGHTNAME));
      }
    } else {
      coneTarget = false;
      cubeTarget = false;
      conePose2d = null;
      cubePose2d = null;
      coneConfidence = 0.0;
      cubeConfidence = 0.0;
    }
  }

  public void doDetection() {
    detectionOn = true;
  }

  public void stopDetection() {
    detectionOn = false;
  }

  public boolean hasCubeTarget() {
    return cubeTarget;
  }

  public Translation2d getClosestCubeTarget() {
    return cubePose2d;
  }

  public boolean hasConeTarget() {
    return coneTarget;
  }

  public void setPieceMode(PieceMode mode) {
    currentPieceMode = mode;

    // set the pipeline to match the scoring mode.
    if (currentPieceMode == PieceMode.CUBE) {
      // FIXME: verify pipeline indices
      pipeline.setDouble(1);
    } else if (currentPieceMode == PieceMode.CONE) {
      pipeline.setDouble(0);
    }
  }

  public Translation2d getClosestConeTarget() {
    return conePose2d;
  }

  private LimelightTarget_Detector[] fetchTargetDetector() {
    if (llresults != null
        && llresults.targetingResults != null
        && llresults.targetingResults.targets_Detector != null) {
      return llresults.targetingResults.targets_Detector;
    }

    return null;
  }

  private void processLlResults(LimelightResults llresults2) {
    LimelightTarget_Detector[] detections = fetchTargetDetector();
    double cubeHeight = 1000;
    double coneHeight = 1000;

    coneTarget = cubeTarget = false;

    if (detections != null) {
      // System.out.println("Targets detected - " + detections.length);
      for (LimelightTarget_Detector detection : detections) {
        // System.out.println("   Target - " + detection.className + " Area: " + detection.ta);
        if (CONE_LABEL.equals(detection.className)) {
          if (detection.ty < coneHeight) {
            conePose2d = new Translation2d(detection.tx, detection.ty);
            coneHeight = detection.ty;
            coneConfidence = detection.confidence;
            coneTarget = true;
          }
        } else if (CUBE_LABEL.equals(detection.className)) {
          if (detection.ty < cubeHeight) {
            cubePose2d = new Translation2d(detection.tx, detection.ty);
            cubeHeight = detection.ty;
            cubeConfidence = detection.confidence;
            cubeTarget = true;
          }
        }
      }
    }
  }
}
