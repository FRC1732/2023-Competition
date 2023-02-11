/*
 * Initially from https://github.com/Mechanical-Advantage/SwerveDevelopment
 */

package frc.lib.team3061.gyro;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO {
  private final AHRS gyro;

  public GyroIONavX() {
    gyro = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.positionDeg = gyro.getYaw() * -1; // degrees
    inputs.velocityDegPerSec = gyro.getRate(); // degrees per second
  }
}
