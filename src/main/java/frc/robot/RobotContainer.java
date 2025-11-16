// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveTrain.AbsoluteFieldDrive;
import frc.robot.commands.Elevator.ElevatorCommand;
import frc.robot.commands.Intake.AdjustCoral;
import frc.robot.commands.Intake.HelpIntakeCommand;
import frc.robot.commands.Intake.IntakeCommand;
import frc.robot.commands.Intake.OuttakeCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ElevatorSubsystem.elevatorState;
import frc.robot.subsystems.IntakeSubsystem.intakeState;
import frc.robot.subsystems.LedSubsystem.LedState;
import frc.robot.subsystems.Popper.popperState;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.FlippingUtil.FieldSymmetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  // The robot's subsystems are defined here... and created when the RobotContainer is constructed
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final Popper m_popper = new Popper();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final DriveTrain m_driveTrain = new DriveTrain(new File(Filesystem.getDeployDirectory(),"swerve"));
  private final LedSubsystem m_leds = new LedSubsystem();

   // Defines a SendableChooser for SmartDashboard
  private final SendableChooser<Command> m_autoChooser;
 
  // Defining the controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = 
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  

  /** The constructor for the RobotContainer. Creates subsystems, OI devices, and commands. */
  public RobotContainer() {
    
      //Putting Subsystem Data on the Smartdashboard
    if (Constants.Verbose) {
      SmartDashboard.putData("Popper Data", m_popper);
      SmartDashboard.putData("Elevator Data", m_elevatorSubsystem);
      SmartDashboard.putData("Intake Data", m_intakeSubsystem);
      SmartDashboard.putData("DriveTrain Data", m_driveTrain);
    }
  
    FlippingUtil.fieldSizeX = 17.55;
    FlippingUtil.fieldSizeY = 8.05;
    FlippingUtil.symmetryType = FieldSymmetry.kMirrored;

    // Configure the trigger bindings
    configureBindings();

    // Register Named Commands

    //#TODO Need to define the algea removal Command sequence

    NamedCommands.registerCommand(
      "L3 Raise", Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.L3))
                              .andThen(Commands.run(() -> m_elevatorSubsystem.updatePosition())
                              .until(
                                      () -> Math.abs(m_elevatorSubsystem.getGoalPosition() - 
                                      m_elevatorSubsystem.getPosition()) < Constants.Elevator.Position_Error
                                                      )));
    NamedCommands.registerCommand(
      "L2 Raise", Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.L2))
                              .andThen(Commands.run(() -> m_elevatorSubsystem.updatePosition())
                              .until(
                                      () -> Math.abs(m_elevatorSubsystem.getGoalPosition() - 
                                      m_elevatorSubsystem.getPosition()) < Constants.Elevator.Position_Error
                                          )));
    NamedCommands.registerCommand(
      "L2 Algea Prep", Commands.runOnce(() -> m_popper.setPopperState(popperState.L2Plus_SPIN))
                                    .andThen(Commands.run(() -> m_popper.updatePosition())
                                    .until(
                                            () -> Math.abs(m_popper.getGoalPosition() - 
                                            m_popper.getPopperPosition()) < Constants.Popper.Position_Error
                                                            )));
    NamedCommands.registerCommand("Stow Popper", Commands.runOnce(() -> m_popper.setPopperState(popperState.Start)).
                                  andThen(Commands.run(() -> m_popper.updatePosition()).until(
                                    () -> Math.abs(m_popper.getGoalPosition() - m_popper.getPopperPosition()) < Constants.Popper.Position_Error
                                  )));
    NamedCommands.registerCommand(
      "L2 Algea", Commands.runOnce(() -> m_popper.setPopperState(popperState.L2Plus_SPIN))
                              .andThen(Commands.run(() -> m_popper.updatePosition())
                              .until(
                                      () -> Math.abs(m_popper.getGoalPosition() - 
                                      m_popper.getPopperPosition()) < Constants.Popper.Position_Error
                                                      )));
    NamedCommands.registerCommand(
      "L3 Algea", Commands.runOnce(() -> m_popper.setPopperState(popperState.L3Plus_SPIN))
                              .andThen(Commands.run(() -> m_popper.updatePosition())
                              .until(
                                      () -> Math.abs(m_popper.getGoalPosition() - 
                                      m_popper.getPopperPosition()) < Constants.Popper.Position_Error
                                                      )));
    NamedCommands.registerCommand(
      "Load", Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.Load))
                          .alongWith(Commands.run(() -> m_elevatorSubsystem.updatePosition())
                          .until(
                                  () -> Math.abs(m_elevatorSubsystem.getGoalPosition() - 
                                  m_elevatorSubsystem.getPosition()) < Constants.Elevator.Position_Error
                                )));
    NamedCommands.registerCommand(
      "autoIntake", new IntakeCommand(m_intakeSubsystem, m_leds).andThen(new AdjustCoral(m_intakeSubsystem)));
    NamedCommands.registerCommand("autoScore", new OuttakeCommand(m_intakeSubsystem, m_leds));

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
  public void configureBindings() {
    //Drivetrain Bindings
    if(Robot.isSimulation()) {
      
      m_driveTrain.setDefaultCommand(new AbsoluteFieldDrive(m_driveTrain, () -> -m_driverController.getLeftY(), 
                                                                () -> -m_driverController.getLeftX(),
                                                                () -> -m_driverController.getRightX(),
                                                                () -> -m_driverController.getRightY(), 
                                                                m_driverController.x().negate()));

      //Turns off an error about the joystick not being connected.
      DriverStation.silenceJoystickConnectionWarning(true);
      
    } else 
    {
      
      m_driveTrain.setDefaultCommand(new AbsoluteFieldDrive(m_driveTrain, () -> -m_driverController.getLeftY(),
                                                                () -> -m_driverController.getLeftX(), 
                                                                () -> -m_driverController.getRightY(),
                                                                () -> -m_driverController.getRightX(),
                                                                () -> m_driverController.getRightTriggerAxis()<0.5));
    }
    m_driverController.start().onTrue(Commands.runOnce(() -> m_driveTrain.zeroGyroWithAlliance()));
    
   
    //This method is called in the teleopInit, once the DriverStation object is created and has posted information. 
     
    //Auto driving methods and bindings
    // These bindings should not happen for demo conditions.
    if (Constants.NotDemo) {
      //#TODO- Finish placing targets and confirm orientations

      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        //The Red location sides
        //Left side Coral Station 
        m_driverController
                          .x()
                          .whileTrue(m_driveTrain.driveToPose(new Pose2d(new Translation2d(15.963, 0.799), 
                                    new Rotation2d(Units.degreesToRadians(120)))));
        //Right side Coral Station 
        m_driverController
                          .b()
                          .whileTrue(m_driveTrain.driveToPose(new Pose2d(new Translation2d(15.963, 7.266), 
                                    new Rotation2d(Units.degreesToRadians(225)))));
      
        //Reef settings
        m_driverController
              .pov(0)
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.R_21)));
              //.whileTrue(m_driveTrain.driveToPose(Constants.Target.R_10));
        m_driverController
              .pov(0)
              .and(m_driverController.a())
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.L_21)));
              //.whileTrue(m_driveTrain.driveToPose(Constants.Target.L_10));

        m_driverController
              .pov(45)
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.R_22)));
              //.whileTrue(m_driveTrain.driveToPose(Constants.Target.R_9));
        m_driverController
              .pov(45)
              .and(m_driverController.a())
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.L_22)));
              //.whileTrue(m_driveTrain.driveToPose(Constants.Target.L_9));

        m_driverController
              .pov(135)
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.R_17)));
              //.whileTrue(m_driveTrain.driveToPose(Constants.Target.R_8));
        m_driverController
              .pov(135)
              .and(m_driverController.a())
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.L_17)));
              //.whileTrue(m_driveTrain.driveToPose(Constants.Target.L_8));

        m_driverController
              .pov(180)
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.R_18)));
              //.whileTrue(m_driveTrain.driveToPose(Constants.Target.R_7));
        m_driverController
              .pov(180)
              .and(m_driverController.a())
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.L_18)));
              //.whileTrue(m_driveTrain.driveToPose(Constants.Target.L_7));

        m_driverController
              .pov(225)
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.R_19)));
              //.whileTrue(m_driveTrain.driveToPose(Constants.Target.R_6));
        m_driverController
              .pov(225)
              .and(m_driverController.a())
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.L_19)));
              //.whileTrue(m_driveTrain.driveToPose(Constants.Target.L_6));

        m_driverController
              .pov(315)
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.R_20)));
              //.whileTrue(m_driveTrain.driveToPose(Constants.Target.R_11));
        m_driverController
              .pov(315)
              .and(m_driverController.a())
              .whileTrue(m_driveTrain.driveToPose(FlippingUtil.flipFieldPose(Constants.Target.L_20)));
}
      else {
        //The blue location sides
        //Right side Coral Station 
        m_driverController
                          .b()
                          .whileTrue( m_driveTrain.driveToPose(new Pose2d(new Translation2d(1.588, 0.799), 
                                    new Rotation2d(Units.degreesToRadians(60)))));
        //Left side Coral Station 
        m_driverController
                          .x()
                          .whileTrue( m_driveTrain.driveToPose(new Pose2d(new Translation2d(1.588, 7.266), 
                                  new Rotation2d(Units.degreesToRadians(300)))));
      

      //Reef settings 

        m_driverController
                          .pov(0)
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.R_21));
        m_driverController
                          .pov(0)
                          .and(m_driverController.a())
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.L_21));

        m_driverController
                          .pov(45)
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.R_22));
        m_driverController
                          .pov(45)
                          .and(m_driverController.a())
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.L_22));

        m_driverController
                          .pov(135)
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.R_17));
        m_driverController
                          .pov(135)
                          .and(m_driverController.a())
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.L_17));

        m_driverController
                          .pov(180)
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.R_18));
        m_driverController
                          .pov(180)
                          .and(m_driverController.a())
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.L_18));

        m_driverController
                          .pov(225)
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.R_19));
        m_driverController
                          .pov(225)
                          .and(m_driverController.a())
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.L_19));
        
        m_driverController
                          .pov(315)
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.R_20));
        m_driverController
                          .pov(315)
                          .and(m_driverController.a())
                          .whileTrue(m_driveTrain.driveToPose(Constants.Target.L_20));
        }
    }
     
    //Elevator Bindings
    /*Default command updates position continuously until goal is met - Although it is not clear that the until means anything
      given that the command restarts immediatly after.
    */
      m_elevatorSubsystem.setDefaultCommand(Commands.run(()-> m_elevatorSubsystem.updatePosition(),m_elevatorSubsystem));
      //Elevator State Setters
      m_operatorController.x().onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.Load)));
      m_operatorController.a().onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.L1)));
      m_operatorController.b().onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.L2)));
      m_operatorController.y().onTrue(Commands.runOnce(() -> m_elevatorSubsystem.setState(elevatorState.L3)));  

      //Allows manual control of elevator with pressing the right Trigger and the left Joystick
      m_operatorController.rightTrigger(0.9)
                          .whileTrue(new ElevatorCommand(m_elevatorSubsystem, () -> m_operatorController.getLeftY()));
    
      //Popper Binding
     
    /*
     * Default Popper command is meant to continuously update the position of the popper arm
    */
      m_popper.setDefaultCommand(Commands.run(() -> m_popper.updatePosition(), m_popper));
        
    //Popper Movement
    //L3 Position
    m_operatorController.povUp()
                  .onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.L3)));
    //L3 Position + Spin
                  m_operatorController.povUp()
                  .and(m_operatorController.leftBumper())
                  .onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.L3_SPIN)));
    //L3+ Position
                  m_operatorController.povUp()
                  .and(m_operatorController.leftTrigger(0.5))
                  .onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.L3Plus)));
    //L3+ Position + Spin
    m_operatorController.povUp()
                  .and(m_operatorController.leftBumper())
                  .and(m_operatorController.leftTrigger(0.5))
                  .onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.L3Plus_SPIN)));
    //L2 Position
    m_operatorController.povLeft()
                  .onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.L2)));
    //L2 Position + Spin
    m_operatorController.povLeft()
                  .and(m_operatorController.leftBumper())
                  .onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.L2_SPIN)));
    //L2+ Position
    m_operatorController.povLeft()
                  .and(m_operatorController.leftTrigger(0.5))
                  .onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.L2Plus)));
    //L2+ Position + Spin
    m_operatorController.povLeft()
                  .and(m_operatorController.leftBumper())
                  .and(m_operatorController.leftTrigger(0.5))
                  .onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.L2Plus_SPIN)));
    //Popper Down     
    m_operatorController.povDown()
                  .onTrue(Commands.runOnce(()-> m_popper.setPopperState(popperState.Start)));
    //Popper Manual Control
    m_operatorController.povRight()
                        .whileTrue(m_popper.rock(() -> m_operatorController.getRightY()));
    //Popper Manual Control + Spin
    m_operatorController.povRight()
                        .and(m_operatorController.leftBumper())
                        .whileTrue(m_popper.rockAndRoll(() -> m_operatorController.getRightY(),() -> 0.2));
                        
    //Means of moving the Popper arm to desired position.
    m_operatorController.back().onTrue(Commands.runOnce(() -> m_elevatorSubsystem.zeroEncoders()));
    m_operatorController.start().onTrue(Commands.runOnce(() -> m_popper.popperReset()));
    
    //Intake Bindings- Note switch to driver controller.
    m_intakeSubsystem.setDefaultCommand(Commands.runOnce(() -> m_intakeSubsystem.stopIntake(), m_intakeSubsystem));

    m_driverController.leftBumper()
                .whileTrue(new OuttakeCommand(
                            m_intakeSubsystem,
                            m_leds))
                .onFalse(Commands.runOnce(
                          () -> m_intakeSubsystem.stopIntake()));
    m_driverController.rightBumper()
                .whileTrue(new IntakeCommand(m_intakeSubsystem, m_leds)
                          .andThen(new AdjustCoral(m_intakeSubsystem)))
                .onFalse(Commands.runOnce(
                          ()-> m_intakeSubsystem.stopIntake()));
    /*
    The goal here is to provide an efficient way to move the coral back and forth
    without accidently ejecting the coral out the back by stopping once the sensor state changes.
    */
    m_operatorController.rightBumper()
                .onTrue(new HelpIntakeCommand(
                            m_intakeSubsystem,
                            m_leds,
                            () -> m_operatorController.getLeftY()))
                .onFalse(Commands.runOnce(
                          () -> m_intakeSubsystem.stopIntake()));
   
   
    //For setting the pose based on vision
    m_driverController.back().onTrue(Commands.runOnce(() -> m_driveTrain.getVisionPose()));
     
    System.out.println("End of configure Bindings");
      
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
  
    public boolean readyToGo(){
      return m_intakeSubsystem.getIntatkeState() == intakeState.HasCoral && m_elevatorSubsystem.getSensor();
    }

    public void goodToGo(){
      m_leds.setState(LedState.HasCoral);
    }
}
