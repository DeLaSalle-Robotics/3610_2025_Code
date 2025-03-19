// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Swerve {
    public static final double MAX_SPEED = 4.5;
    public static final double LOOP_TIME = 0.13;
    public static final double ROBOT_MASS = 45.5;
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class Intake {
    public static final int motorId = 16;
    public static final int sensorId = 1;
  }

  public static class  Elevator {
    public static final int elevatorMotorId = 18;
    public static final int sensorId = 0;

    public static final double holdValue = -0.04;
    public static final double threshold = 50;
    //Values of positions are rotations of the motor shaft.
    public static final double Start_Position = 0;
    public static final double L1_Position = 0;
    public static final double L2_Position = -9.1;
    public static final double L3_Position = -28.5;
    public static final double Load_Position = -48.0;
    
  }
  public static class Popper {
    /*NOTE: The popper position values are from the builtin 
     * SparkMax encoder (i.e. not the through-bore). These do
     * not account for the gearing occurring with the sprockets.
     */
    public static double Start_Position = 0;
    public static double L2_Position = 40;
    public static double L3_Position = 70;
    public static int popperRotateID = 14;
    public static int popperSpinnerID = 15;
    public static int popperEncoderChannelA = 2;
    public static int popperEncoderChannelB = 3;
    public static double popperSpinnerSpeed = 0.5;
    public static double minAngle = 0;//EncoderValue Not actuly an Angle
    public static double maxAngle = 730;//EncoderValue Not actuly an Angle
    
  }

  public static class Led {
    public static final int numLeds = 120;
    public static final int ledPwm = 9;
    public static final double rainbowShiftSpeed = 5.0;
  }
}
