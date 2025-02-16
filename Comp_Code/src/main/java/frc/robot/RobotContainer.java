// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrain.AbsoluteDrive;
import frc.robot.commands.DriveTrain.AbsoluteFieldDrive;
import frc.robot.subsystems.*;
import frc.robot.commands.DriveTrain.*;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final DriveTrain driveTrain = new DriveTrain(new File(Filesystem.getDeployDirectory(),"swerve"));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  Trigger aButton = m_driverController.a();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    if(Robot.isSimulation()) {
      driveTrain.setDefaultCommand(new AbsoluteFieldDrive(driveTrain, () -> m_driverController.getLeftX(), 
                                                                () -> m_driverController.getLeftY(),
                                                                () -> m_driverController.getRightX()));
      // driveTrain.setDefaultCommand(new AbsoluteDrive(driveTrain, () -> m_driverController.getLeftX(), 
      //                                                           () -> m_driverController.getLeftY(),
      //                                                           () -> m_driverController.getRightX(),
      //                                                           () -> m_driverController.getRightY()));
      
    } else 
    {
      driveTrain.setDefaultCommand(new AbsoluteFieldDrive(driveTrain, () -> m_driverController.getLeftX(), 
                                                                () -> m_driverController.getLeftY(),
                                                                () -> m_driverController.getRightX()));
      // driveTrain.setDefaultCommand(new AbsoluteDrive(driveTrain, () -> m_driverController.getLeftX(), 
      //                                                           () -> m_driverController.getLeftY(),
      //                                                           () -> m_driverController.getRightX(),
      //                                                           () -> m_driverController.getRightY()));
                                                              }
                                                              
    m_driverController.a().onTrue( driveTrain.driveToPose(new Pose2d(new Translation2d(5.509, 2.425), 
                                                          new Rotation2d(Units.degreesToRadians(105)))
                                                          ));
    m_driverController.b().onTrue(new TurnToAngle(driveTrain, new Pose2d(new Translation2d(0, 0), 
                                                          new Rotation2d(Units.degreesToRadians(105)))
                                                          ));
                                                          
    }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    try{
      PathPlannerPath path = PathPlannerPath.fromPathFile("Far to H.path");
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Problem..." + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
    
}
