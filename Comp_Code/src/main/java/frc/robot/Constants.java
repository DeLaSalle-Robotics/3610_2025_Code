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
    public static final double Start_Position = -1;
    public static final double L1_Position = -1;
    public static final double Load_Position = -9.6;
    public static final double L2_Position = -29;
    public static final double L3_Position = -49.0;
    public static final double Position_Error = 0.2;
    
  }
  public static class Popper {
    /*NOTE: The popper position values are from the builtin 
     * SparkMax encoder (i.e. not the through-bore). These do
     * not account for the gearing occurring with the sprockets.
     */
    public static final double Start_Position = -0.263671875;
    public static final double L2_Position = -13.20849609375;
    public static final double L2Plus_Position = -13;
    public static final double L3_Position = -13.0;
    public static final double L3Plus_Position = -16.0;
    public static int popperRotateID = 14;
    public static int popperSpinnerID = 15;
    public static int popperEncoderChannelA = 2;
    public static int popperEncoderChannelB = 3;
    public static double popperSpinnerSpeed = 0.2;
    public static double minAngle = 0;//EncoderValue Not actuly an Angle
    public static double maxAngle = 730;//EncoderValue Not actuly an Angle
    public static double PopperStep = -0.1;
    public static double maxPosition = -13;
    public static double minPosition = -10;
    public static final double Position_Error = 1.0;
  }

  public static class Led {
    public static final int numLeds = 120;
    public static final int ledPwm = 9;
    public static final double rainbowShiftSpeed = 5.0;
  }

  public static class Climber {
    public static final int climberID = 17;

    public static final double Start_Position = 0;
    public static final double Down_Position = -31.50146484375;
    public static final double Climb_Position = 25.166015625;


  }
}
