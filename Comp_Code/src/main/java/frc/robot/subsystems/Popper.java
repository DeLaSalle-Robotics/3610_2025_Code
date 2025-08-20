// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Popper extends SubsystemBase {
  //private final SparkMax Rotater = new SparkMax(Constants.Popper.popperRotateID, MotorType.kBrushless);
  private final TalonFX Rotater = new TalonFX(Constants.Popper.popperRotateID);
  private final SparkMax Spinner = new SparkMax(Constants.Popper.popperSpinnerID, MotorType.kBrushless);
  
 
  //Some sort of gyro scope to set grasper position
  private final Encoder intakeArmEncoder = new Encoder(Constants.Popper.popperEncoderChannelA,Constants.Popper.popperEncoderChannelB, false, EncodingType.k4X);
  //private final DutyCycleEncoder intakeArmEncoder = new DutyCycleEncoder(2);
  public static final double armOffset = 83.0;
  
  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0).withSlot(0);
  double initialPoint;
 
  public enum popperState{
  Start,
  L2,
  L2Plus,
  L3,
  L3Plus,
  L2_SPIN,
  L2Plus_SPIN,
  L3_SPIN,
  L3Plus_SPIN
 }

 private popperState currentState;

 //Constructing the Subsystem
 public Popper() {
    Rotater.getConfigurator().apply(new TalonFXConfiguration());
    Rotater.setNeutralMode(NeutralModeValue.Brake);
    initialPoint = intakeArmEncoder.get();
    currentState = popperState.Start;
    //Configuring the Popper Rotation Motor
    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kG = 0;
    slot0Configs.kA = 0;
    slot0Configs.kV = 0;
    // PID runs on position
    slot0Configs.kP = 1.5;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    //Current Limits
    var limitConfigs = talonFXConfigs.CurrentLimits;
    limitConfigs.StatorCurrentLimit = 40;
    limitConfigs.StatorCurrentLimitEnable = true;


    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 40; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 80; // 160 rps/s acceleration -> 0.5 to reach max speed
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 second)

    Rotater.getConfigurator().apply(talonFXConfigs, 0.05);
    Rotater.setPosition(0);
    intakeArmEncoder.setDistancePerPulse(0.1758); //Degrees/Pulse
 }

//Methods===================
/**
 * Provides a means of controlling the Popper rotation motor.
 * Should use with extreme caution to move the arm to a reset
 * position.
 * @param speed
 */
public void popperMove(Double speed) {
  //Creating a save speed to avoid damage while reseting arm position.
  double safeSpeed = 0.5 * speed;
  if (Constants.Verbose) {SmartDashboard.putNumber("RockSpeed", safeSpeed);}
  
  Rotater.set(safeSpeed);
}
/**
 * Provides a means to reset the arm position. Used in conjuction with 
 * popperMove method.
 */
public void popperReset() {
  this.setPopperPosition(0);
}

public double getPopperPosition() {
   return Rotater.getRotorPosition().getValueAsDouble();
}

/**
 * Moves the Popper rotator arm to desired state using MotionMagic™
 * controller. 
 * @param setPoint
 */
public void setPopperPosition(double setPoint){
  //m_controller.setReference(setPoint, ControlType.kMAXMotionPositionControl);
  Rotater.setControl(m_motmag.withPosition(setPoint));

}

/**
 * Takes a popperState and makes it the desired state.
 * @param newState
 */
public void setPopperState(popperState newState){
  this.currentState = newState;
}
public void PopperSpinL3(){
  Spinner.set(-Constants.Popper.popperSpinnerSpeed);
}
public void popperSpin(){
  Spinner.set(-Constants.Popper.popperSpinnerSpeed);
}
public void popperSpinStop(){
  Spinner.set(0);
}
public void updatePosition(){
  switch (this.currentState) {
    case Start -> {
      this.setPopperPosition(Constants.Popper.Start_Position);
      this.popperSpinStop();
    }
    case L2 -> {
      this.setPopperPosition(Constants.Popper.L2_Position);
      this.popperSpinStop();
    }
    case L3 -> {
      this.setPopperPosition(Constants.Popper.L3_Position);
      this.popperSpinStop();
    }
    case L2Plus -> {
      this.setPopperPosition(Constants.Popper.L2Plus_Position);
      this.popperSpinStop();
    }
    case L3Plus -> {
      this.setPopperPosition(Constants.Popper.L3Plus_Position);
      this.popperSpinStop();
    }
    case L2_SPIN -> {
      this.setPopperPosition(Constants.Popper.L2_Position);
      this.popperSpin();
    }
    case L3_SPIN -> {
      this.setPopperPosition(Constants.Popper.L3_Position);
      this.popperSpin();
    }
    case L2Plus_SPIN -> {
      this.setPopperPosition(Constants.Popper.L2Plus_Position);
      this.popperSpin();
    }
    case L3Plus_SPIN -> {
      this.setPopperPosition(Constants.Popper.L3Plus_Position);
      this.popperSpin();
    }
    default ->{
      this.setPopperPosition(Constants.Popper.Start_Position);
      this.popperSpinStop();
    }
  }
}

public double getGoalPosition(){
  switch (this.currentState) {
    case Start -> {
      return(Constants.Popper.Start_Position);
    }
    case L2 -> {
      return(Constants.Popper.L2_Position);
    }
    case L3 -> {
      return(Constants.Popper.L3_Position);
    }
    case L2Plus ->{
      return(-14.2);
    }
    case L3Plus -> {
      return(Constants.Popper.L3Plus_Position);
    }
    default ->{
      return(Constants.Popper.Start_Position);
    }
  }
}
//Commands====================================
/**
   * This command will allow spinning and moving of the Popper arm.
   *
   * @return a rockAndRoll Command
   */
  public Command rockAndRoll(DoubleSupplier speed,DoubleSupplier spinSpeed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          popperMove(speed.getAsDouble());
          if(spinSpeed.getAsDouble() >= 0.2){
            Spinner.set(0.2);  
          }
          else{Spinner.set(spinSpeed.getAsDouble());}
        });
  }
  
/**
   * This command will allow spinning and moving of the Popper arm.
   *
   * @return a rockAndRoll Command
   */
  public Command rock(DoubleSupplier speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    
    return run(
        () -> {
          popperMove(speed.getAsDouble());
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    
    if (Constants.Verbose) {SmartDashboard.putNumber("PopperPosition", getPopperPosition());
    SmartDashboard.putNumber("PopperVoltage", Rotater.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("PopperCurrent", Rotater.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putString("PopperState",this.currentState.toString());
    SmartDashboard.putNumber("PopperGoal", getGoalPosition());
    SmartDashboard.putNumber("PopperError", Math.abs(getPopperPosition() - getGoalPosition()));
  }
    
    //SmartDashboard.putNumber("P", rotaterConfig);
    
    
    //Note this will run the motors to pre set positions. Do not activate until tested.
    
    //this.updatePosition();


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
