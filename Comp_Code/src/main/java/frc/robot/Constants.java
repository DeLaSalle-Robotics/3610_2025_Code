// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

  public static boolean Verbose = true;
  public static boolean NotDemo = false;

  public static class Swerve {
    public static final double MAX_SPEED = 5;
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
    public static final int backSensorId = 4;
    }

  public static class  Elevator {
    public static final int elevatorMotorId = 18;
    public static final int sensorId = 0;

    public static final double holdValue = 0.04;
    public static final double threshold = 50;
    //Values of positions are rotations of the motor shaft.
    public static final double Start_Position = 1;
    public static final double L1_Position = 1;
    public static final double Load_Position = 10;
    public static final double L2_Position = 28;
    public static final double L3_Position = 47.5;
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
    public static final int popperRotateID = 14;
    public static final int popperSpinnerID = 15;
    public static final int popperEncoderChannelA = 2;
    public static final int popperEncoderChannelB = 3;
    public static final double popperSpinnerSpeed = 0.2;
    public static final double minAngle = 0;//EncoderValue Not actually an Angle
    public static final double maxAngle = 730;//EncoderValue Not actually an Angle
    public static final double PopperStep = -0.1;
    public static final double maxPosition = -13;
    public static final double minPosition = -10;
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

  public static class Target {
      public static final Pose2d R_Front_Red = new Pose2d(new Translation2d(14.441, 4.214),
                                                        new Rotation2d(0));
      public static final Pose2d R_LeftFront_Red =  new Pose2d(new Translation2d(13.851, 3.132),
                                                            new Rotation2d(Units.degreesToRadians(300)));
      public static final Pose2d R_LeftBack_Red = new Pose2d(new Translation2d(12.525, 2.830),
                                                          new Rotation2d(Units.degreesToRadians(240)));
      public static final Pose2d R_Back_Red = new Pose2d(new Translation2d(11.735, 3.863),
                                                      new Rotation2d(Units.degreesToRadians(180)));
      public static final Pose2d R_RightBack_Red = new Pose2d(new Translation2d(12.385, 5.008),
                                                            new Rotation2d(Units.degreesToRadians(120)));
      public static final Pose2d R_RightFront_Red = new Pose2d(new Translation2d(13.492, 5.188),
                                                            new Rotation2d(Units.degreesToRadians(60)));
      public static final Pose2d L_Front_Red = new Pose2d(new Translation2d(14.442, 3.925),
                                                        new Rotation2d(0));
      public static final Pose2d L_LeftFront_Red = new Pose2d(new Translation2d(13.496, 2.908),
                                                            new Rotation2d(Units.degreesToRadians(300)));
      public static final Pose2d L_LeftBack_Red = new Pose2d(new Translation2d(12.165, 3.032),
                                                          new Rotation2d(Units.degreesToRadians(240)));
      public static final Pose2d L_Back_Red = new Pose2d(new Translation2d(11.735, 4.221),
                                                      new Rotation2d(Units.degreesToRadians(180)));
      public static final Pose2d L_RightBack_Red = new Pose2d(new Translation2d(12.552, 5.171),
                                                            new Rotation2d(Units.degreesToRadians(120)));
      public static final Pose2d L_RightFront_Red = new Pose2d(new Translation2d(13.926, 5.029),
                                                            new Rotation2d(Units.degreesToRadians(60)));
      public static final Pose2d R_Front_Blue = new Pose2d(new Translation2d(2.888,3.788),
                                                        new Rotation2d(Units.degreesToRadians(0)));
      public static final Pose2d R_LeftFront_Blue =  new Pose2d(new Translation2d(3.582, 5.223),
                                                        new Rotation2d(Units.degreesToRadians(120)));
      public static final Pose2d R_LeftBack_Blue = new Pose2d(new Translation2d(5.055, 5.331),
                                                        new Rotation2d(Units.degreesToRadians(60)));
      public static final Pose2d R_Back_Blue = new Pose2d(new Translation2d(5.868, 4.248),
                                                        new Rotation2d(0));
      public static final Pose2d R_RightBack_Blue = new Pose2d(new Translation2d(5.346, 2.990),
                                                        new Rotation2d(Units.degreesToRadians(300)));
      public static final Pose2d R_RightFront_Blue = new Pose2d(new Translation2d(3.951, 2.821),
                                                        new Rotation2d(Units.degreesToRadians(240)));
      public static final Pose2d L_Front_Blue = new Pose2d(new Translation2d(2.888, 4.151),
                                                        new Rotation2d(Units.degreesToRadians(0)));
      public static final Pose2d L_LeftFront_Blue = new Pose2d(new Translation2d(3.951, 5.538),
                                                        new Rotation2d(Units.degreesToRadians(120)));
      public static final Pose2d L_LeftBack_Blue = new Pose2d(new Translation2d(5.335, 5.154),
                                                        new Rotation2d(Units.degreesToRadians(60)));
      public static final Pose2d L_Back_Blue = new Pose2d(new Translation2d(5.868, 3.886),
                                                        new Rotation2d(0));
      public static final Pose2d L_RightBack_Blue = new Pose2d(new Translation2d(5.068, 2.821),
                                                        new Rotation2d(Units.degreesToRadians(300)));
      public static final Pose2d L_RightFront_Blue = new Pose2d(new Translation2d(3.582, 2.990),
                                                        new Rotation2d(Units.degreesToRadians(240)));
    

  }
}