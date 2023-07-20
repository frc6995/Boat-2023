package frc.robot.util.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

public class DriveUtil {
    public static WheelSpeeds curvatureDriveIK(
        double xSpeed, double zRotation, boolean allowTurnInPlace) {
      xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
    //   zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
  
      double leftSpeed;
      double rightSpeed;
  
      if (allowTurnInPlace) {
        leftSpeed = xSpeed - zRotation;
        rightSpeed = xSpeed + zRotation;
      } else {
        leftSpeed = xSpeed - Math.abs(xSpeed) * zRotation;
        rightSpeed = xSpeed + Math.abs(xSpeed) * zRotation;
      }
  
      // Desaturate wheel speeds
      double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
      if (maxMagnitude > 1.0) {
        leftSpeed /= maxMagnitude;
        rightSpeed /= maxMagnitude;
      }
  
      return new WheelSpeeds(leftSpeed, rightSpeed);
    }
}
