// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PopperCommand;
import frc.robot.commands.DriveTrain.AbsoluteDrive;
import frc.robot.commands.DriveTrain.AbsoluteFieldDrive;
import frc.robot.commands.DriveTrain.DriveToTarget;
import frc.robot.commands.Popper.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ClimberSubsystem.climberState;
import frc.robot.subsystems.ElevatorSubsystem.elevatorState;
import frc.robot.subsystems.Popper.popperState;
import frc.robot.commands.IntakeCommand;

import java.io.File;
import java.time.Period;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final Popper m_popper = new Popper();
  //private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final DriveTrain m_driveTrain = new DriveTrain(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final LedSubsystem m_leds = new LedSubsystem();

   // Allows picking autonomous routines from SmartDashboard
  private final SendableChooser<Command> m_autoChooser;
  
  DoubleSubscriber xTarget;
  DoubleSubscriber yTarget;
  DoubleSubscriber thetaTarget;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = 
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  Trigger aButton = m_driverController.a();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    xTarget = table.getDoubleTopic("xTar").subscribe(0);
    yTarget = table.getDoubleTopic("yTar").subscribe(0);
    thetaTarget = table.getDoubleTopic("thetaTar").subscribe(0);
      
      //Putting Subsystem Data on the Smartdashboard
    SmartDashboard.putData("Popper Data", m_popper);
    SmartDashboard.putData("Elevator Data", m_elevatorSubsystem);
    //SmartDashboard.putData("Climber Data", m_climber);
    SmartDashboard.putData("Intake Data", m_intakeSubsystem);

  
    // Configure the trigger bindings
    configureBindings();

    // Register Named Commands
    NamedCommands.registerCommand("L3 Raise", Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.L3)).
                                              andThen(Commands.run(() -> m_elevatorSubsystem.updatePosition()).until(
                                    () -> Math.abs(m_elevatorSubsystem.getGoalPosition() - m_elevatorSubsystem.getPosition()) < Constants.Elevator.Position_Error
                                  )));
    NamedCommands.registerCommand("L2 Algea Prep", Commands.runOnce(() -> m_popper.setPopperState(popperState.L2)).
                                  andThen(Commands.run(() -> m_popper.updatePosition()).until(
                                    () -> Math.abs(m_popper.getGoalPosition() - m_popper.getPopperPosition()) < Constants.Popper.Position_Error
                                  )));
    NamedCommands.registerCommand("Stow Popper", Commands.runOnce(() -> m_popper.setPopperState(popperState.Start)).
                                  andThen(Commands.run(() -> m_popper.updatePosition()).until(
                                    () -> Math.abs(m_popper.getGoalPosition() - m_popper.getPopperPosition()) < Constants.Popper.Position_Error
                                  )));
    NamedCommands.registerCommand("L2 Algea", new PopperL2Command(m_popper));
    NamedCommands.registerCommand("L3 Algea", new PopperL3Command(m_popper).andThen(Commands.run(() -> m_popper.updatePosition()).until(
      () -> Math.abs(m_popper.getGoalPosition() - m_popper.getPopperPosition()) < Constants.Popper.Position_Error
                                  )));
    NamedCommands.registerCommand("Load", Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.Load)).
                                              andThen(Commands.run(() -> m_elevatorSubsystem.updatePosition()).until(
                                    () -> Math.abs(m_elevatorSubsystem.getGoalPosition() - m_elevatorSubsystem.getPosition()) < Constants.Elevator.Position_Error
                                  )));
    NamedCommands.registerCommand("autoIntake", new IntakeCommand(m_intakeSubsystem, m_leds,() -> -0.5, true));
    NamedCommands.registerCommand("autoScore", new IntakeCommand(m_intakeSubsystem, m_leds, () ->  -0.5, true));

    // Build an auto chooser
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    SmartDashboard.putString("Robot State", "Have Coral");
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
      
      m_driveTrain.setDefaultCommand(new AbsoluteFieldDrive(m_driveTrain, () -> m_driverController.getLeftX(), 
                                                                () -> m_driverController.getLeftY(),
                                                                () -> m_driverController.getRightX(), 
                                                                m_driverController.x()));
      // driveTrain.setDefaultCommand(new AbsoluteDrive(driveTrain, () -> m_driverController.getLeftX(), 
      //                                                           () -> m_driverController.getLeftY(),
      //                                                           () -> m_driverController.getRightX(),
      //                                                           () -> m_driverController.getRightY()));
      
    } else 
    {
      
      m_driveTrain.setDefaultCommand(new AbsoluteFieldDrive(m_driveTrain, () -> -m_driverController.getLeftY(),
                                                                () -> -m_driverController.getLeftX(), 
                                                                () -> -m_driverController.getRightX(),
                                                                () -> m_driverController.getRightTriggerAxis()<0.5));
  
      // driveTrain.setDefaultCommand(new AbsoluteDrive(driveTrain, () -> m_driverController.getLeftX(), 
      //                                                           () -> m_driverController.getLeftY(),
      //                                                           () -> m_driverController.getRightX(),
      //                                                           () -> m_driverController.getRightY()));
    }
    
    /*
    * Auto Driving Commands- not currently working
    m_driverController.a().onTrue( m_driveTrain.driveToPose(new Pose2d(new Translation2d(1.588, 0.799), 
                                                          new Rotation2d(Units.degreesToRadians(40)))
                                                          ));

    m_driverController.b().onTrue(Commands.defer(m_driveTrain.driveSupplier(), Set.of(m_driveTrain)));
    */  
      
    //Popper Binding
    
    //m_popper.setDefaultCommand(new PopperCommand(m_popper));
    m_popper.setDefaultCommand(m_popper.rockAndRoll(() -> m_operatorController.getRightY(), () -> m_operatorController.getLeftY()));
    m_operatorController.back().whileTrue(new PopperCommand(m_popper));
    
    //m_operatorController.rightBumper().onTrue(m_popper.rockAndRoll(m_operatorController.getLeftY()));
    
      //m_driverController.leftBumper().whileTrue(m_popper.rock(() -> (m_driverController.getLeftTriggerAxis())));
      //m_driverController.rightBumper().whileTrue(m_popper.rock(()->(-m_driverController.getRightTriggerAxis())));
      
      //m_driverController.b().whileTrue(m_popper.rockAndRoll(-0.11,-0.5));
      //m_driverController.a().whileTrue(m_popper.rockAndRoll(0,0.5));
      m_driverController.start().onTrue(Commands.runOnce(() -> m_driveTrain.zeroGyro()));
  
      m_driverController.povUp().onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.L3)));
      m_driverController.povLeft().onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.L2)));
      m_driverController.povDown().onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.Start)));
      
      m_driverController.leftBumper().onTrue(new PopperL2Remove(m_popper));
      m_driverController.rightBumper().onTrue(new PopperL3Remove(m_popper));

      //Elevator Bindings
      
      m_elevatorSubsystem.setDefaultCommand(Commands.runOnce(()-> m_elevatorSubsystem.updatePosition(),m_elevatorSubsystem).until(
        () -> Math.abs(m_elevatorSubsystem.getGoalPosition() - m_elevatorSubsystem.getPosition()) < Constants.Elevator.Position_Error
      ));
      

      m_operatorController.x().onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.Load)));
      m_operatorController.a().onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.L1)));
      m_operatorController.b().onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.L2)));
      m_operatorController.y().onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.L3)));  
      m_operatorController.leftBumper().whileTrue(new IntakeCommand(m_intakeSubsystem, m_leds,() -> -0.3,true));
      //m_operatorController.rightBumper().whileTrue(new IntakeCommand(m_intakeSubsystem, m_leds,() -> -0.5,false));
      m_operatorController.povDown().whileTrue(new IntakeCommand(m_intakeSubsystem, m_leds, () -> 0.8, true));
      
      
      m_operatorController.start().whileTrue(new ElevatorCommand(m_elevatorSubsystem,()->m_operatorController.getRightY()));
      
      
      //m_operatorController.x().onTrue(new IntakeCommand(m_intakeSubsystem, m_leds,() -> -0.5,true));
      //m_operatorController.y().whileTrue(new IntakeCommand(m_intakeSubsystem, m_leds,() -> -0.5,false));
      //m_operatorController.povRight().onTrue(Commands.run(()-> m_climber.setState(climberState.Out)));
      //m_operatorController.povUp().onTrue(Commands.run(()-> m_climber.setState(climberState.Start)));
      //m_operatorController.povUp().onTrue(Commands.run(()-> m_climber.setState(climberState.In)));
                                                            }
                                                          
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
      //return m_ShooterSubsystem.autoShooter(m_IntakeSubsystem);
    }
  
    
}
