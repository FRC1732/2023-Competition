// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LimelightScoring extends SubsystemBase {
  // limelight hostname: http://limelight-scoring.local:5801
  private final String LIMELIGHTNAME = "limelight-scoring";
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

  private ProfiledPIDController _thetaController;
  private GenericEntry rotationP;
  private GenericEntry rotationI;
  private GenericEntry rotationD;

  private final Pose2d EMPTY_POSE2D = new Pose2d();
  private LimelightHelpers.LimelightResults llresults;
  private boolean havePosition = false;
  private Pose2d position = EMPTY_POSE2D;

  private ScoringMode currentScoringMode = ScoringMode.Undefined;

  /** Creates a new Limelight. */
  public LimelightScoring() {
    configureNetworkTableEntries();
    configureShuffleBoard();
  }

  public void on() {
    // ledMode.setNumber(LL_LEDSTATE_ON);
  }

  public void off() {
    // ledMode.setNumber(LL_LEDSTATE_OFF);
  }

  private void configureNetworkTableEntries() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    ledMode = table.getEntry("ledMode");
    camMode = table.getEntry("camMode");
  }

  private void configureShuffleBoard() {
    ShuffleboardTab tab;
    LLFeed = new HttpCamera("limelight", "http://10.17.32.11:5800/stream.mjpg");
    server2 = CameraServer.getServer("serve_USB Camera 0");
    LLFeed.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    tab = Shuffleboard.getTab("limelight");
    tab.addNumber("LED Mode", ll_ledModeSupplier);
    tab.addNumber("tv - Valid Targets", ll_tvSupplier);
    tab.addNumber("tx - Horiz Offset", ll_txSupplier);
    tab.addNumber("ty - Vert Offset", ll_tySupplier);
    tab.addBoolean("Target Acquired", ll_hasTarget);
    rotationP =
        tab.add("rotation p", 0).withWidget(BuiltInWidgets.kTextView).withSize(1, 1).getEntry();
    rotationI =
        tab.add("rotation I", 0).withWidget(BuiltInWidgets.kTextView).withSize(1, 1).getEntry();
    rotationD =
        tab.add("rotation D", 0).withWidget(BuiltInWidgets.kTextView).withSize(1, 1).getEntry();
  }

  DoubleSupplier ll_ledModeSupplier =
      new DoubleSupplier() {
        @Override
        public double getAsDouble() {
          return 0;
        }
      };

  DoubleSupplier ll_tvSupplier =
      new DoubleSupplier() {
        @Override
        public double getAsDouble() {
          return r_tv;
        }
      };

  DoubleSupplier ll_txSupplier =
      new DoubleSupplier() {
        @Override
        public double getAsDouble() {
          return r_tx;
        }
      };

  DoubleSupplier ll_tySupplier =
      new DoubleSupplier() {
        @Override
        public double getAsDouble() {
          return r_ty;
        }
      };

  DoubleSupplier ll_taSupplier =
      new DoubleSupplier() {
        @Override
        public double getAsDouble() {
          return r_ta;
        }
      };

  public void nullifyPID() {
    _thetaController = null;
  }

  public DoubleSupplier rotation =
      new DoubleSupplier() {
        @Override
        public double getAsDouble() {
          if (!hasTarget()) return 0;
          double targetRad = Math.toRadians(getTx());
          if (_thetaController == null) {
            var profileConstraints =
                new TrapezoidProfile.Constraints(
                    Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                    Constants.MAX_ANGULAR_ACCELERATION * Math.PI / 180 * 5);
            _thetaController = new ProfiledPIDController(13, 0, 1, profileConstraints);
            _thetaController.enableContinuousInput(Math.PI * -1, Math.PI);
            _thetaController.reset(targetRad);
          }

          return _thetaController.calculate(targetRad, 0);
        }
      };

  BooleanSupplier ll_hasTarget =
      new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
          return hasTarget();
        }
      };

  @Override
  public void periodic() {
    // read and store values periodically
    llresults = LimelightHelpers.getLatestResults("");

    try {
      position = llresults.targetingResults.getBotPose2d();
    } catch (NullPointerException e) {
      position = EMPTY_POSE2D;
    }

    havePosition = EMPTY_POSE2D.equals(position);

    if (havePosition) {
      r_tx = position.getX();
      r_ty = position.getY();
      r_ta = 1;
      r_tv = havePosition ? 1 : 0;
    } else {
      r_tx = -1;
      r_ty = -1;
      r_ta = -1;
      r_tv = -1;
    }

    if (llresults != null) {
      // System.out.println(LimelightHelpers.getJSONDump(""));
    }
  }

  public boolean hasTarget() {
    return r_tv > 0;
  }

  public int getPriorityTag() {
    if (hasTarget()) {
      return (int) llresults.targetingResults.targets_Fiducials[0].fiducialID;
    }

    return -1;
  }

  public boolean isAligned() {
    return Math.abs(getTx()) < 2.5;
  }

  public Double getTx() {
    return r_tx;
  }

  public Double getTy() {
    return r_ty;
  }

  public Pose2d getPose2d() {
    return position;
  }

  public enum ScoringMode {
    AprilTag,
    ReflectiveTape,
    Undefined;
  }
}
