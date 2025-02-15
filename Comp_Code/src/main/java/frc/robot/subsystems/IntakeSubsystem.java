// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor;
    private final DigitalInput sensor;

    private boolean hasCoral;
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new SparkMax(Constants.Intake.motorId, MotorType.kBrushless);
    sensor = new DigitalInput(Constants.Intake.sensorId);
  }
  public void stopIntake() {
    intakeMotor.set(0.0);
  }
  public void startIntake() {
    intakeMotor.set(Constants.Intake.motorSpeed);
  }
  private boolean detectCoral() {
    return sensor.get();
  }
  public boolean getHasCoral() {
    return hasCoral;
  }

  @Override
  public void periodic() {
    boolean has = detectCoral();
    if(has != hasCoral) {
        hasCoral = has;
        SmartDashboard.putBoolean("Has Coral", has);
    }
    if(sensor.get() && !hasCoral) {
        hasCoral = true;
        stopIntake();
    } else {
        hasCoral = false;
    }
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
