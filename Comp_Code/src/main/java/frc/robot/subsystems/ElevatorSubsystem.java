// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorMotor = new TalonFX(Constants.Elevator.elevatorMotorId);;
  private final DigitalInput limitSwitch = new DigitalInput(Constants.Elevator.sensorId);
  private final PowerDistribution powerDistHub = new PowerDistribution();
  /*
  BooleanSubscriber goingUp;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("elevator");
  */

  public enum elevatorState{
    Start,
    L1,
    L2,
    L3,
    Load,
    Stuck
  }
    private elevatorState currentState;
    final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0).withSlot(0);

    public ElevatorSubsystem() {
    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    
    //elevatorMotor.setInverted(true);
    currentState = elevatorState.Start;

    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kG = 0.28;
    slot0Configs.kS = 0.72; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.0; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 5;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration -> 0.5 to reach max speed
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 second)
    
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit=20;
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    elevatorMotor.getConfigurator().apply(talonFXConfigs, 0.05);

    if(getSensor()){
      zeroEncoders();
    }
    //goingUp = table.getBooleanTopic("goingUp").subscribe(false);
  }

  public void stopElevator(){
    elevatorMotor.set(Constants.Elevator.holdValue);
  }

  public void runElevator(double speed){
  elevatorMotor.set(speed);
  }

  public boolean getSensor(){
    return limitSwitch.get();
  }

  public void zeroEncoders(){
    //elevatorMotor.setSelectedSensorPosition(0);
    elevatorMotor.setPosition(0);
  }

  public double getPosition(){
    return elevatorMotor.getRotorPosition().getValueAsDouble();
  }

  public double getElevatorCurrent(){
    return powerDistHub.getCurrent(Constants.Elevator.pdhChannel);
  }

  public void setPosition(double spot){
    elevatorMotor.setControl(m_motmag.withPosition(spot));
  }

  public void setState(elevatorState state){
    this.currentState = state;
  }

  public void updatePosition(){
    switch (this.currentState) {
      case Start -> {
        this.setPosition(Constants.Elevator.Start_Position);
      }
      case Load -> {
        this.setPosition(Constants.Elevator.Load_Position);
      }
      case L1 -> {
        this.setPosition(Constants.Elevator.L1_Position);
      }
      case L2 -> {
        this.setPosition(Constants.Elevator.L2_Position);
      }
      case L3 -> {
        this.setPosition(Constants.Elevator.L3_Position);
      }
      case Stuck -> {
        this.setPosition(this.getPosition());
      }
      default ->{
        this.setPosition(Constants.Elevator.Load_Position);
      }
    }
  }

    public double getGoalPosition(){
      switch (this.currentState) {
        case Start -> {
          return(Constants.Elevator.Start_Position);
        }
        case Load -> {
          return(Constants.Elevator.Load_Position);
        }
        case L1 -> {
          return(Constants.Elevator.L1_Position);
        }
        case L2 -> {
          return(Constants.Elevator.L2_Position);
        }
        case L3 -> {
          return(Constants.Elevator.L3_Position);
        }
        default ->{
          return(Constants.Elevator.Load_Position);
        }
      }
    }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command holdCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(() -> stopElevator());
  }

    @Override
  public void periodic() {
    //This should zero the encoder and reset elevator state to Start if the bottom limit is hit.
    
    /*
    if(getSensor()){
      this.zeroEncoders();
      this.setState(elevatorState.Start);
    }
      */
    if (Constants.Verbose) {
      SmartDashboard.putNumber("Elevator Encoder", this.getPosition());
      SmartDashboard.putNumber("Elevator Voltage", elevatorMotor.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Elevator Motor Current", this.getElevatorCurrent());
    } 

    //If motor current is too great - Need to check to see what this number should be
    if (this.getElevatorCurrent() > 20) {
      //Stop the motor
      elevatorMotor.set(0);
      //Redefine the set point
      this.setState(elevatorState.Stuck);
      
    }
    //this.updatePosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
