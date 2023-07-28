// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.drive.DriveUtil;
import frc.robot.util.sparkmax.SparkMax;

import  static frc.robot.Constants.DrivebaseConstants.*;
import static frc.robot.util.sparkmax.SparkMax.check;
import static frc.robot.util.sparkmax.SparkMax.config;
import static frc.robot.util.sparkmax.SparkMax.SparkMaxInitializers.*;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class DrivebaseS extends SubsystemBase {

  private SparkMax frontLeftMotor = new SparkMax(CAN_ID_FRONT_LEFT, MotorType.kBrushless);
  private SparkMax frontRightMotor = new SparkMax(CAN_ID_FRONT_RIGHT, MotorType.kBrushless);
  private SparkMax backLeftMotor = new SparkMax(CAN_ID_BACK_LEFT, MotorType.kBrushless);
  private SparkMax backRightMotor = new SparkMax(CAN_ID_BACK_RIGHT, MotorType.kBrushless);
  private SlewRateLimiter fwdBackSlewRate = new SlewRateLimiter(2);
  /** Creates a new DrivebaseS. */
  public DrivebaseS() {
    frontLeftMotor.withSettings(
      brake(),
      currentLimit(SPARK_MAX_CURRENT_LIMIT),
      invert()
    ).withFollower(
      backLeftMotor.withSettings(
        brake()
      )
    );

    frontRightMotor.withSettings(
      brake(),
      currentLimit(SPARK_MAX_CURRENT_LIMIT)
    ).withFollower(
      backRightMotor.withSettings(
        brake()
      )
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double fwdBack, double turn, boolean arcade) {
    fwdBack = MathUtil.applyDeadband(fwdBack, 0.05);
    turn = MathUtil.applyDeadband(turn, 0.05);
    fwdBack *= 1;
    fwdBack = fwdBackSlewRate.calculate(fwdBack);
    
    WheelSpeeds speeds;
    if (!arcade) {
      speeds = DriveUtil.curvatureDriveIK(fwdBack, turn * 2, false);
    } else {
     speeds = DifferentialDrive.arcadeDriveIK(fwdBack, turn, false);
    }
    SmartDashboard.putNumber("left", speeds.left);
    frontLeftMotor.setVoltage(speeds.left * 12);
    frontRightMotor.setVoltage(speeds.right * 12);
  }

  public Command driveLandC(DoubleSupplier fwdBack, DoubleSupplier turn) {
    return this.run(
      ()->{
        drive(fwdBack.getAsDouble(), turn.getAsDouble(), false);
      }
    );
  }

  public Command driveWaterC(DoubleSupplier fwdBack, DoubleSupplier turn) {
    return this.run(
      ()->{
        drive(fwdBack.getAsDouble(), turn.getAsDouble(), true);
      }
    );
  }
}
