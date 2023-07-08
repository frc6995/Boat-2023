// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import  static frc.robot.Constants.DrivebaseConstants.*;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DrivebaseS extends SubsystemBase {

  private CANSparkMax frontLeftMotor = new CANSparkMax(CAN_ID_FRONT_LEFT, MotorType.kBrushless);
  private CANSparkMax frontRightMotor = new CANSparkMax(CAN_ID_FRONT_RIGHT, MotorType.kBrushless);
  private CANSparkMax backLeftMotor = new CANSparkMax(CAN_ID_BACK_LEFT, MotorType.kBrushless);
  private CANSparkMax backRightMotor = new CANSparkMax(CAN_ID_BACK_RIGHT, MotorType.kBrushless);

  /** Creates a new DrivebaseS. */
  public DrivebaseS() {
    configureC().schedule();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double fwdBack, double turn) {
    WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(fwdBack, turn, false);
    SmartDashboard.putNumber("left", speeds.left);
    frontLeftMotor.setVoltage(speeds.left * 12);
    frontRightMotor.setVoltage(speeds.right * 12);
  }

  public Command driveLandC(DoubleSupplier fwdBack, DoubleSupplier turn) {
    return this.run(
      ()->{
        drive(fwdBack.getAsDouble(), turn.getAsDouble());
      }
    );
  }

  private Command configureC() {
    return sequence(
      runOnce(()->{
        frontLeftMotor.restoreFactoryDefaults();
        frontRightMotor.restoreFactoryDefaults();
        backLeftMotor.restoreFactoryDefaults();
        backRightMotor.restoreFactoryDefaults();
      }),
      waitSeconds(2),
      runOnce(()->{
          backLeftMotor.follow(frontLeftMotor);
          backRightMotor.follow(frontRightMotor);

          frontRightMotor.setInverted(true);

          frontLeftMotor.setSmartCurrentLimit(30);
          frontRightMotor.setSmartCurrentLimit(30);
          frontLeftMotor.setIdleMode(IdleMode.kBrake);
          frontLeftMotor.setIdleMode(IdleMode.kBrake);
      }),
      waitSeconds(2),
      runOnce(()->{
        frontLeftMotor.burnFlash();
        frontRightMotor.burnFlash();
        backLeftMotor.burnFlash();
        backRightMotor.burnFlash();
      }))
      .ignoringDisable(true)
    ;

  }
}
