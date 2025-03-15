// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
  private final SparkMax Rotater = new SparkMax(Constants.Popper.popperRotateID, MotorType.kBrushless);
  private final SparkMax Spinner = new SparkMax(Constants.Popper.popperSpinnerID, MotorType.kBrushless);
 
  //Some sort of gyro scope to set grasper position
  private final Encoder intakeArmEncoder = new Encoder(Constants.Popper.popperEncoderChannelA,
                                                        Constants.Popper.popperEncoderChannelB, false, EncodingType.k4X);
  //private final DutyCycleEncoder m_abEncoder = new DutyCycleEncoder(2);
  public static final double armOffset = 83.0;


  //Simulation
  private final DCMotor Rotater_sim = DCMotor.getNEO(Constants.Popper.popperRotateID);
  private final double m_armReduction = 5.0; // gear Ratio
  private final double m_armMass = 2; //in kg
  private final double m_armLength = Units.inchesToMeters(24);
  private double priorArmVelocity = 0.0;
  private static double armPositionRad = Math.PI/4;

 
 //Declaring the Subsystem \/
 public Popper() {
  
  intakeArmEncoder.setDistancePerPulse(0.1758); //Degrees/Pulse
 }

//Methods===================

public void PopperMove(Double speed) {
  if (Math.abs(speed) > 0.1) {
    double popperPosition = this.getPopperPosition();
    if (popperPosition > Constants.Popper.maxAngle  && speed < 0){
      Rotater.set(0);
    } else if (popperPosition < Constants.Popper.minAngle && speed > 0) {
      Rotater.set(0);
    } else {
      Rotater.set(speed);
    }
  } else {
    Rotater.set(0);
  }
}

public double getPopperPosition() {

  return intakeArmEncoder.getDistance() - armOffset;
}

public void PopperSpin(){
  Spinner.set(Constants.Popper.popperSpinnerSpeed);
}

//Commands====================================
/**
   * This command will allow spinning and moving of the Popper arm.
   *
   * @return a rockAndRoll Command
   */
  public Command rockAndRoll(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          PopperMove(speed);
          PopperSpin();
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
