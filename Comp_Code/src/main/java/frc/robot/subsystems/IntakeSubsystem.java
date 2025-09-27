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

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor;
    private final DigitalInput sensor;
    private final DigitalInput backSensor;
    private SparkMaxConfig motorConfig;
    private boolean backSensorBroken = false;
    
    public enum intakeState{
        Empty,
        Loading,
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



    public void periodicSetIntakeState() {
        
        if(sensor.get() && backDetectCoral()){
            this.currentState = intakeState.HasCoral;
        }
        else if (!sensor.get() && !backDetectCoral()){
            this.currentState = intakeState.Empty;
        }

        if(this.currentState == intakeState.LoadingBrokenBackSensor){
            backSensorBroken = true;
        }
    }

    public boolean frontDetectCoral(){
        return sensor.get();
    }

    public boolean backDetectCoral() {    
        return !backSensor.get();
    }

    @Override
    public void periodic() {
        if(this.currentState == intakeState.Loading){
            
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
