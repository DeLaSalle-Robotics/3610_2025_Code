// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX elevatorMotor;
  private final DigitalInput limitSwitch;
  BooleanSubscriber goingUp;
  
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("elevator");
  
    public ElevatorSubsystem() {
    elevatorMotor = new TalonFX(Constants.Elevator.elevatorMotorId);
    elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    limitSwitch = new DigitalInput(Constants.Elevator.sensorId);

    goingUp = table.getBooleanTopic("goingUp").subscribe(false);
  }

  public void stopElevator(){
    elevatorMotor.set(-0.02);
  }

  public void runElevator(double speed){
  elevatorMotor.set(speed);
  }

  public boolean getSensor(){
    return limitSwitch.get();
  }

  public void zeroEncoders(){
    //elevatorMotor.setSelectedSensorPosition(0);
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
      stopElevator();
    }
    //SmartDashboard.putNumber("Elevator Encoder", elevatorMotor.getSelectedSensorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
