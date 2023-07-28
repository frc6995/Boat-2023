// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.sparkmax.SparkMax;

import  static frc.robot.Constants.ArmConstants.*;
import static frc.robot.util.sparkmax.SparkMax.check;
import static frc.robot.util.sparkmax.SparkMax.config;
import static frc.robot.util.sparkmax.SparkMax.SparkMaxInitializers.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
public class ArmS extends SubsystemBase {

  private SparkMax frontRightMotor = new SparkMax(CAN_ID_FRONT, MotorType.kBrushless);
  private SparkMax backRightMotor = new SparkMax(CAN_ID_BACK, MotorType.kBrushless);
  private SparkMaxAbsoluteEncoder m_encoder = frontRightMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.1, 0.1);
  private PIDController m_pidController = new PIDController(2, 0, 0);
  private BooleanSupplier isWaterMode = ()->false;
  /** Creates a new DrivebaseS. */
  public ArmS(BooleanSupplier isWaterMode) {

    frontRightMotor.withSettings(
      brake(),
      currentLimit(SPARK_MAX_CURRENT_LIMIT),
      statusFramePeriod(PeriodicFrame.kStatus5, 10),
      absEncoderPositionConversion(2*Math.PI)
    ).withFollower(
      backRightMotor.withSettings(
        brake()
      )
    );
    this.isWaterMode = isWaterMode;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("arm", getAngle());
    SmartDashboard.putNumber("front", frontRightMotor.getAppliedOutput() * 12);
    SmartDashboard.putNumber("back", backRightMotor.getAppliedOutput() * 12);
    SmartDashboard.putNumber("current", frontRightMotor.getOutputCurrent());
    // This method will be called once per scheduler run
  }
  private double getHoldingVoltage() {
    return 0.6 * Math.cos(m_encoder.getPosition());
  }
  /**
   * Returns the angle from the absolute encoder. If the arm is below horizontal,
   * REV reports values near 2pi, so convert the range between 2pi and 2pi-reverseSoftLimit
   * into a range between 0 and reverseSoftLimit
   * @return
   */
  private double getAngle() {
    double position = m_encoder.getPosition();
    if (position > (1.5* Math.PI) ){position -= 2*Math.PI;}
    return position;
  }

  private void setVoltage(double volts) {
    SmartDashboard.putNumber("inVolts", volts);
    if (getAngle() > FORWARD_SOFT_LIMIT) {volts = Math.min(volts, 0);}
    else if (getAngle() < getBackSoftLimit()) {volts = Math.max(volts, 0);}
    double voltage = volts + getHoldingVoltage();
    double backSoftLimit = isWaterMode.getAsBoolean() ? -Math.PI/4 : BACKWARD_SOFT_LIMIT;

    SmartDashboard.putNumber("armVolts", voltage);
    frontRightMotor.setVoltage(voltage);
  }
  private double getBackSoftLimit() {
    double backSoftLimit = isWaterMode.getAsBoolean() ? -Math.PI/4 : BACKWARD_SOFT_LIMIT;
    return backSoftLimit;
  }
  private void setPosition(double position) {
    setVoltage(m_pidController.calculate( getAngle(), position));
  } 

  public Command manualC(DoubleSupplier speed) {
    return run(()->{
        setVoltage(speed.getAsDouble() * -3);
    });
  }

  public Command positionC(DoubleSupplier position) {
    return run(()->{
        setPosition(MathUtil.clamp(position.getAsDouble(), getBackSoftLimit(), FORWARD_SOFT_LIMIT));
    });
  }

}
