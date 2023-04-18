package frc.lib.team3061.gyro;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class GyroIoADIS16470 implements GyroIO {
  private ADIS16470_IMU imu;

  public GyroIoADIS16470() {
    imu = new ADIS16470_IMU();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = imu.isConnected();
    inputs.positionDeg = -1 * imu.getAngle(); // degrees
    inputs.velocityDegPerSec = -1 * imu.getRate(); // degrees per second
  }

  public void calibrate() {
    imu.calibrate();
  }

  public double xComplementary() {
    return imu.getXComplementaryAngle();
  }

  public double yComplementary() {
    return imu.getYComplementaryAngle();
  }
}
