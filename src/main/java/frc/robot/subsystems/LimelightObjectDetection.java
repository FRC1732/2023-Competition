// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class LimelightObjectDetection extends SubsystemBase {
  // limelight hostname: http://limelight-objectd.local:5801
  private NetworkTable table;
  private NetworkTableEntry llData_camerastream;
  private NetworkTableEntry tv;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry ledMode;
  private NetworkTableEntry camMode;

  private double r_tv;
  private double r_tx;
  private double r_ty;
  private double r_ta;

  private HttpCamera LLFeed;
  private VideoSink server2;
  private int cameraStream = 0;

  private LimelightHelpers.LimelightResults llresults;
  private boolean havePosition = false;

  /** Creates a new Limelight. */
  public LimelightObjectDetection() {
    configureNetworkTableEntries();
    configureShuffleBoard();
  }

  private void configureNetworkTableEntries() {
    table = NetworkTableInstance.getDefault().getTable("ObjectDetection");
  }

  private void configureShuffleBoard() {
    ShuffleboardTab tab;
    LLFeed = new HttpCamera("limelight", "http://10.17.32.12:5800/stream.mjpg");
    LLFeed.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    tab = Shuffleboard.getTab("limelight");
  }

  @Override
  public void periodic() {
    // read and store values periodically
    llresults = LimelightHelpers.getLatestResults("");

    if (llresults != null) {
      // System.out.println(LimelightHelpers.getJSONDump(""));
    }
  }

  public boolean hasTarget() {
    return r_tv > 0;
  }
}
