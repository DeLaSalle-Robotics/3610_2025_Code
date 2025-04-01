// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
  private double setPoint = 0.0;
  
  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0).withSlot(0);
  double initialPoint;
 public enum popperState{
  Start,
  L2,
  L2Plus,
  L3,
  L3Plus
 }

 //Declaring the Subsystem \/
 public Popper() {
  Rotater.getConfigurator().apply(new TalonFXConfiguration());
  Rotater.setNeutralMode(NeutralModeValue.Brake);
      initialPoint = intakeArmEncoder.get();
  //intakeArmEncoder.setDistancePerPulse(0.1758); //Degrees/Pulse
   
  var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kG = 0;
    slot0Configs.kA = 0;
    slot0Configs.kV = 0;
    // PID runs on position
    slot0Configs.kP = 1.5;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 40; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 80; // 160 rps/s acceleration -> 0.5 to reach max speed
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 second)

    Rotater.getConfigurator().apply(talonFXConfigs, 0.05);
    Rotater.setPosition(0);
  intakeArmEncoder.setDistancePerPulse(0.1758); //Degrees/Pulse
 }
private popperState currentState = popperState.Start;

//Methods===================

public void PopperMove(Double speed) {
  if (Constants.Verbose) {SmartDashboard.putNumber("RockSpeed", speed);}
  Rotater.set(speed);
  /*
  if (Math.abs(speed) > 0.01) {
    double popperPosition = this.getPopperPosition();
    if (popperPosition > Constants.Popper.maxAngle-50  && speed < 0){
      Rotater.set(0);
    } else if (popperPosition < Constants.Popper.minAngle+50 && speed > 0) {
      Rotater.set(0);
    } else {
    }
  } else {
    Rotater.set(0);
  }
  */
}

public double getPopperPosition() {
  /* This code uses the throughbore encoder, but that cannot be sent directly to the sparkMax
  *   without a specific adapter from Rev (REV-11-1881-PK2). Consider if built-in encoder is 
  *  not sufficiently sensitive to control the arm.
  */
  //return intakeArmEncoder.getDistance() - armOffset;
  return Rotater.getRotorPosition().getValueAsDouble();
}

public void setPopperPosition(double setPoint){
  //m_controller.setReference(setPoint, ControlType.kMAXMotionPositionControl);
  Rotater.setControl(m_motmag.withPosition(setPoint));

}

public void zeroArm(){
  Rotater.setPosition(0);
}

public void setPopperState(popperState newState){
  this.currentState = newState;
}
public void PopperSpinL3(){
  Spinner.set(-Constants.Popper.popperSpinnerSpeed);
}
public void PopperSpinL2(){
  Spinner.set(-Constants.Popper.popperSpinnerSpeed);
}
public void PopperSpinStop(){
  Spinner.set(0);
}
public void updatePosition(){
  switch (this.currentState) {
    case Start -> {
      this.setPopperPosition(Constants.Popper.Start_Position);
    }
    case L2 -> {
      this.setPopperPosition(Constants.Popper.L2_Position);
    }
    case L3 -> {
      this.setPopperPosition(Constants.Popper.L3_Position);
    }
    case L2Plus -> {
      this.setPopperPosition(Constants.Popper.L2Plus_Position);
    }
    case L3Plus -> {
      this.setPopperPosition(Constants.Popper.L3Plus_Position);
    }
    default ->{
      this.setPopperPosition(Constants.Popper.Start_Position);
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
          PopperMove(speed.getAsDouble());
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
          PopperMove(speed.getAsDouble());
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
