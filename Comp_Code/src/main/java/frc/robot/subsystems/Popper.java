// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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
  private final SparkMax Rotater = new SparkMax(Constants.Popper.popperRotateID, MotorType.kBrushless);
  private final SparkMax Spinner = new SparkMax(Constants.Popper.popperSpinnerID, MotorType.kBrushless);
  SparkMaxConfig rotaterConfig = new SparkMaxConfig();
 
  //Some sort of gyro scope to set grasper position
  private final Encoder intakeArmEncoder = new Encoder(Constants.Popper.popperEncoderChannelA,Constants.Popper.popperEncoderChannelB, false, EncodingType.k4X);
  //private final DutyCycleEncoder intakeArmEncoder = new DutyCycleEncoder(2);
  public static final double armOffset = 83.0;
  private double setPoint = 0.0;

  SparkClosedLoopController m_controller = Rotater.getClosedLoopController();

  //Simulation
  private final DCMotor Rotater_sim = DCMotor.getNEO(Constants.Popper.popperRotateID);
  private final double m_armReduction = 5.0; // gear Ratio
  private final double m_armMass = 2; //in kg
  private final double m_armLength = Units.inchesToMeters(24);
  private double priorArmVelocity = 0.0;
  private static double armPositionRad = Math.PI/4;
  double initialPoint;
 public enum popperState{
  Start,
  L2,
  L3
 }

 //Declaring the Subsystem \/
 public Popper() {
  rotaterConfig.idleMode(IdleMode.kBrake);
  Rotater.configure(rotaterConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
 
  initialPoint = intakeArmEncoder.get();
  //intakeArmEncoder.setDistancePerPulse(0.1758); //Degrees/Pulse
   
  
  SparkMaxConfig config = new SparkMaxConfig();
  config.closedLoop.p(0.1);
  config.closedLoop.i(0);
  config.closedLoop.d(0);
  config.closedLoop.maxMotion.maxVelocity(4.00)
                            .maxAcceleration(12)
                            .allowedClosedLoopError(0.1);


  intakeArmEncoder.setDistancePerPulse(0.1758); //Degrees/Pulse
 }
private popperState currentState;

//Methods===================

public void PopperMove(Double speed) {
  SmartDashboard.putNumber("RockSpeed", speed);
  if (Math.abs(speed) > 0.01) {
    double popperPosition = this.getPopperPosition();
    if (popperPosition > Constants.Popper.maxAngle-50  && speed < 0){
      Rotater.set(0);
    } else if (popperPosition < Constants.Popper.minAngle+50 && speed > 0) {
      Rotater.set(0);
    } else {
      Rotater.set(speed);
    }
  } else {
    Rotater.set(0);
  }
}

public double getPopperPosition() {
  /* This code uses the throughbore encoder, but that cannot be sent directly to the sparkMax
  *   without a specific adapter from Rev (REV-11-1881-PK2). Consider if built-in encoder is 
  *  not sufficiently sensitive to control the arm.
  */
  //return intakeArmEncoder.getDistance() - armOffset;
  return Rotater.getEncoder().getPosition();
}

public void setPopperPosition(double setPoint){
  m_controller.setReference(setPoint, ControlType.kMAXMotionPositionControl);
  
}

public void setPopperState(popperState newState){
  this.currentState = newState;
}
public void PopperSpin(){
  Spinner.set(Constants.Popper.popperSpinnerSpeed);
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
    default ->{
      this.setPopperPosition(Constants.Popper.Start_Position);
    }
  }
}
//Commands====================================
/**
   * This command will allow spinning and moving of the Popper arm.
   *
   * @return a rockAndRoll Command
   */
  public Command rockAndRoll(double speed,double spinSpeed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          PopperMove(speed);
          PopperSpin(spinSpeed);
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
    SmartDashboard.putNumber("PopperPosition", getPopperPosition());
    //Not this will run the motors to pre set positions. Do not activate until tested.
    
    //this.updatePosition();


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
