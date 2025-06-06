// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor;
    private final DigitalInput sensor;

    private boolean hasCoral = false;

    /** Creates a new ExampleSubsystem. */
    public IntakeSubsystem() {
        intakeMotor = new SparkMax(Constants.Intake.motorId, MotorType.kBrushless);
        sensor = new DigitalInput(Constants.Intake.sensorId);
    }

    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    public void startIntake(double speed) {
        intakeMotor.set(speed);
    }

    public boolean detectCoral() {
        boolean has = sensor.get();
        if (has != hasCoral) {
            hasCoral = has;
            if (Constants.Verbose) {SmartDashboard.putBoolean("Has Coral", has);}
        }
        return hasCoral;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
