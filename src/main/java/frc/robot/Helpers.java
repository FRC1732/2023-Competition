package frc.robot;

public class Helpers {
  // #region Math helpers
  public static boolean DoubleNearEquals(double a, double b, double tolerence) {
    return Math.abs(a - b) < tolerence;
  }

  public static boolean DoubleNearEquals(double a, double b) {
    return DoubleNearEquals(a, b, 10e-7);
  }
  // #endregion
}
