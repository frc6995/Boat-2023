// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmS;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.JetS;
import frc.robot.util.sparkmax.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivebaseS drivebaseS = new DrivebaseS();
  private final JetS jetS = new JetS();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
    
  private boolean isWaterMode = false;

  private Trigger waterModeTrigger = new Trigger(()->isWaterMode);
  private final ArmS armS = new ArmS(waterModeTrigger);

  private Command landWheelC = drivebaseS.driveLandC(
    ()-> m_driverController.getRightTriggerAxis()-m_driverController.getLeftTriggerAxis(),
    ()-> -m_driverController.getLeftX()
  );
  private StringPublisher landWaterPublisher = NetworkTableInstance.getDefault().getStringTopic("/DriverDisplay/label").publish();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    landWaterPublisher.accept("Land");
    SparkMax.burnFlashInSync();
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    waterModeTrigger.onTrue(Commands.runOnce(()->landWaterPublisher.accept("Water")));
    waterModeTrigger.onFalse(Commands.runOnce(()->landWaterPublisher.accept("Land")));
    armS.setDefaultCommand(armS.manualC(m_driverController::getRightY));
    m_driverController.a().onTrue(
      runOnce(()->isWaterMode = !isWaterMode)
    );

    m_driverController.b().toggleOnTrue(armS.positionC(()->0));
    m_driverController.x().toggleOnTrue(armS.positionC(()->Math.PI/4));
    m_driverController.y().toggleOnTrue(armS.positionC(()->-Math.PI/4));
    drivebaseS.setDefaultCommand(drivebaseS.driveLandC(
      ()-> m_driverController.getRightTriggerAxis()-m_driverController.getLeftTriggerAxis(),
      ()-> -m_driverController.getLeftX()));
    waterModeTrigger.whileTrue(
      parallel(
        //drivebaseS.driveLandC(()->0, ()->0),
        drivebaseS.driveWaterC(
          ()-> m_driverController.getRightTriggerAxis()-m_driverController.getLeftTriggerAxis(),
          ()-> -m_driverController.getLeftX()
        ),
        jetS.driveLandC(
          ()-> m_driverController.getRightTriggerAxis()-m_driverController.getLeftTriggerAxis(),
          ()-> -m_driverController.getLeftX()
        )
      )
    );
    
    waterModeTrigger.whileFalse(
      parallel(
        landWheelC,
        jetS.driveLandC(()->{
          if(m_driverController.rightBumper().getAsBoolean()) { return 1;} else {return 0;}
        }
        , ()->0)
      )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return none();
  }
}
