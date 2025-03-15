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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorMotor = new TalonFX(Constants.Elevator.elevatorMotorId);;
  private final DigitalInput limitSwitch = new DigitalInput(Constants.Elevator.sensorId);
  BooleanSubscriber goingUp;
  
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("elevator");
  
  public enum elevatorState{
    Start,
    L1,
    L2,
    L3,
    Load
  }
    private elevatorState currenState;
    final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0).withSlot(0);

    public ElevatorSubsystem() {
    elevatorMotor.getConfigurator().apply(new TalonFXConfiguration());
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    elevatorMotor.setInverted(true);
    currenState = elevatorState.Start;

    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kG = 0.24;
    slot0Configs.kS = 0.0; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.0; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 4.8;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration -> 0.5 to reach max speed
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 second)

    elevatorMotor.getConfigurator().apply(talonFXConfigs, 0.05);


    goingUp = table.getBooleanTopic("goingUp").subscribe(false);
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

  public void setPosition(double spot){
    elevatorMotor.setControl(m_motmag.withPosition(spot));
  }

  public void setState(elevatorState state){
    this.currenState = state;
  }

  public void updatePosition(){
    switch (this.currenState) {
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
      default ->{
        this.setPosition(Constants.Elevator.Load_Position);
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
    if(getSensor() /*&& goingUp.get()*/){
      zeroEncoders();
      runElevator(0.0);
    }
    SmartDashboard.putNumber("Elevator Encoder", this.getPosition());
    this.updatePosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
