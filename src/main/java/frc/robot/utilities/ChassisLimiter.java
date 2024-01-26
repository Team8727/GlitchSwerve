package frc.robot.utilities;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.util.WPIUtilJNI;

public class ChassisLimiter {
  private ChassisSpeeds currentSpeeds = new ChassisSpeeds();

  private final double translationLimit;
  private final double rotationLimit;
  private double prevTime;

  public ChassisLimiter(double translationLimit, double rotationLimit) {
    this.translationLimit = translationLimit;
    this.rotationLimit = rotationLimit;
    prevTime = WPIUtilJNI.now() * 1e-6;
  }

  // Convert a ChassisSpeeds object into a Vector
  private Vector<N2> ChassisSpeedsToVector(ChassisSpeeds chassisSpeeds) {
    return VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
  }

  // Set the translational components of a ChassisSpeeds object to a vector
  private ChassisSpeeds applyVector(ChassisSpeeds chassisSpeeds, Vector<N2> vector) {
    chassisSpeeds.vxMetersPerSecond = vector.get(0, 0);
    chassisSpeeds.vyMetersPerSecond = vector.get(1, 0);
    return chassisSpeeds;
  }

  // Limit the desired nextSpeeds of the chassis to the acceleration limits
  public ChassisSpeeds calculate(ChassisSpeeds nextSpeeds) {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - prevTime;
    prevTime = currentTime;

    var limitedSpeeds = new ChassisSpeeds();

    Vector<N2> velocityDiff =
        new Vector<N2>(
            ChassisSpeedsToVector(nextSpeeds).minus(ChassisSpeedsToVector(currentSpeeds)));

    double speedDiff = velocityDiff.norm();

    double limitFactor =
        (speedDiff == 0)
            ? 0
            : (MathUtil.clamp(
                    speedDiff, -translationLimit * elapsedTime, translationLimit * elapsedTime)
                / speedDiff);

    Vector<N2> velocityVector =
        new Vector<N2>(ChassisSpeedsToVector(currentSpeeds).plus(velocityDiff.times(limitFactor)));

    var limitedAngle =
        MathUtil.clamp(
            nextSpeeds.omegaRadiansPerSecond,
            currentSpeeds.omegaRadiansPerSecond - (rotationLimit * elapsedTime),
            currentSpeeds.omegaRadiansPerSecond + (rotationLimit * elapsedTime));

    limitedSpeeds.omegaRadiansPerSecond = limitedAngle;
    applyVector(limitedSpeeds, velocityVector);

    return limitedSpeeds;
  }

  // Update the current speed of the chassis
  public void update(ChassisSpeeds currentSpeeds) {
    this.currentSpeeds = currentSpeeds;
  }
}
