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
import static frc.robot.Constants.JetConstants.*;

import java.util.function.DoubleSupplier;
public class JetS extends SubsystemBase {

  private final Spark leftJet = new Spark(PWM_LEFT_JET);
  private final Spark rightJet = new Spark(PWM_RIGHT_JET);
  /** Creates a new JetS. */
  public JetS() {}

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
