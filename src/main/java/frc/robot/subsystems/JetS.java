// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sparkmax.SparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.JetConstants.*;
import static frc.robot.util.sparkmax.SparkMax.check;
import static frc.robot.util.sparkmax.SparkMax.config;

import java.util.function.DoubleSupplier;
public class JetS extends SubsystemBase {

  private final SparkMax leftJet = new SparkMax(CAN_ID_LEFT_JET, MotorType.kBrushed);
  private final SparkMax rightJet = new SparkMax(CAN_ID_RIGHT_JET, MotorType.kBrushed);
  /** Creates a new JetS. */
  public JetS() {
    leftJet.withSettings(
      check(spark->spark.setIdleMode(CANSparkMax.IdleMode.kBrake)),
      check(spark->spark.setSmartCurrentLimit(SPARK_MAX_CURRENT_LIMIT)),
      config(spark->spark.setInverted(true))
    );

    rightJet.withSettings(
      check(spark->spark.setIdleMode(CANSparkMax.IdleMode.kBrake)),
      check(spark->spark.setSmartCurrentLimit(SPARK_MAX_CURRENT_LIMIT))
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double fwdBack, double turn) {
    WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(fwdBack, turn, false);
    SmartDashboard.putNumber("leftJet", speeds.left);
    leftJet.set(speeds.left);
    rightJet.set(speeds.right);
  }

  public Command driveLandC(DoubleSupplier fwdBack, DoubleSupplier turn) {
    return this.run(
      ()->{
        drive(fwdBack.getAsDouble(), turn.getAsDouble());
      }
    );
  }
}
