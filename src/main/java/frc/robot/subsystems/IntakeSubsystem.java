// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor;
    private final DigitalInput sensor;
    private final DigitalInput backSensor;
    private SparkMaxConfig motorConfig;
    private boolean backSensorBroken = false;
    
    public enum intakeState{
        Empty,
        Loading,
        LoadingEntry,
        LoadingForward,
        HasCoral,
        LoadingBrokenBackSensor
    }

    private intakeState currentState = intakeState.Empty;

    /** Creates a new ExampleSubsystem. */
    public IntakeSubsystem() {
        intakeMotor = new SparkMax(Constants.Intake.motorId, MotorType.kBrushless);
        sensor = new DigitalInput(Constants.Intake.sensorId);
        backSensor = new DigitalInput(Constants.Intake.backSensorId);
        motorConfig = new SparkMaxConfig();
        motorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);
        

        intakeMotor.configure(motorConfig, null, null);
    }

    public void close(){
        intakeMotor.close();
        sensor.close();
        backSensor.close();
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    public void startIntake(double speed) {

        if(backSensorBroken){
            this.currentState = intakeState.LoadingBrokenBackSensor;
        }
        else{
            this.currentState = intakeState.Loading;
        }
        
        intakeMotor.set(speed);
    }

    public intakeState getIntatkeState(){
        return currentState;
    }

    public void setIntakeState(intakeState state){
        this.currentState = state;
    }


    /**
     * This method examines the intake sensors to determine the coral state.
     *  It is run every cycle.
     */
    public void periodicSetIntakeState() {
        //Test if back sensor is broken before setting other states
        if(this.currentState == intakeState.LoadingBrokenBackSensor){
            this.backSensorBroken = true;
        }

        if (this.backSensorBroken){
            //If the back sensor is misbehaving, state determination ignores it.
            if(frontDetectCoral()){
                this.currentState = intakeState.HasCoral;
            }
            else if (!frontDetectCoral()){
                this.currentState = intakeState.Empty;
            }
        } else{
            if(frontDetectCoral() && backDetectCoral()){
                this.currentState = intakeState.HasCoral;
            }
            else if (!frontDetectCoral() && !backDetectCoral()){
                this.currentState = intakeState.Empty;
            }
            else if (frontDetectCoral() && !backDetectCoral()){
                this.currentState = intakeState.LoadingForward;
            }
            else if (!frontDetectCoral() && backDetectCoral()){
                this.currentState = intakeState.LoadingEntry;
            }
        }
        
    }

    public boolean frontDetectCoral(){
        if (Robot.isReal()){
            return sensor.get();
        } else {
            return SmartDashboard.getBoolean("Front Sensor Sim", false);
        }
    }

    public boolean backDetectCoral() {    
        if (Robot.isReal()) {
            return !backSensor.get();
        } else {
            return SmartDashboard.getBoolean("Back Sensor Sim", false);
        }
    }

    public boolean backSensorTrust(){
        return !this.backSensorBroken;
    }
    
    /**This method takes a boolean value and sets this value to the value of the front intake sensor. */
    public void setSimFrontSensor(boolean value){
        if(value){
            SmartDashboard.putBoolean("Front Sensor Sim", true);
        } else {
            SmartDashboard.putBoolean("Front Sensor Sim", false);
        }
        System.out.println("Set Front Sensor Sim");
    
    }
    /**This method takes a boolean value and sets this value to the value of the front intake sensor. */
    public void setSimBackSensor(boolean value){
        if(value){
            SmartDashboard.putBoolean("Back Sensor Sim", true);
        } else {
            SmartDashboard.putBoolean("Back Sensor Sim", false);
        }
        System.out.println("Set Back Sensor Sim");
           
    }

    public void setSimSensors(boolean front, boolean back) {
        this.setSimBackSensor(back);
        this.setSimFrontSensor(front);
        System.out.println("Set Sim Sensors");
    }
    


    @Override
    public void periodic() {
        periodicSetIntakeState();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation        
    }
}
