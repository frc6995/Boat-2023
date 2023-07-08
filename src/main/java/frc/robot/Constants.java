// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DrivebaseConstants {
    public static final int CAN_ID_FRONT_LEFT = 10;
    public static final int CAN_ID_FRONT_RIGHT = 11;
    public static final int CAN_ID_BACK_LEFT = 12;
    public static final int CAN_ID_BACK_RIGHT = 13;
  }
  public static class JetConstants {
    public static final int PWM_LEFT_JET = 0;
    public static final int PWM_RIGHT_JET = 1;
  }
}
