// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import java.util.List;
import java.util.concurrent.TransferQueue;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AbsoluteFieldDrive extends Command {
  private final DriveTrain swerve;
  private final DoubleSupplier vX, vY, hX, hY;
  private final BooleanSupplier relativeToField;

 /**
   * Used to drive a swerve robot in full field-centric mode.  vX and vY supply translation inputs, where x is
   * torwards/away from alliance wall and y is left/right. headingHorzontal and headingVertical are the Cartesian
   * coordinates from which the robot's angle will be derived— they will be converted to a polar angle, which the robot
   * will rotate to.
   *
   * @param swerve  The swerve drivebase subsystem.
   * @param vX      DoubleSupplier that supplies the x-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive X is away from the alliance wall.
   * @param vY      DoubleSupplier that supplies the y-translation joystick input.  Should be in the range -1 to 1 with
   *                deadband already accounted for.  Positive Y is towards the left wall when looking through the driver
   *                station glass.
   * @param hX DoubleSupplier that supplies the x componsend of the robot's heading angle.
   * @param hY DoubleSupplier that supplies the x componsend of the robot's heading angle.
   */
  public AbsoluteFieldDrive(DriveTrain swerve, DoubleSupplier vX, DoubleSupplier vY,
                            DoubleSupplier hX, DoubleSupplier hY, BooleanSupplier relativeToField)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.hX = hX;
    this.hY = hY;
    this.relativeToField = relativeToField;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d targetHeading = new Translation2d(hX.getAsDouble(),hY.getAsDouble());
    if (targetHeading.getNorm() < 0.3) {
      targetHeading = new Translation2d(1, swerve.getHeading());
    }
    //This method uses the various inputs to return the target velocities.
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                                                      targetHeading.getAngle());
    // Essential removes the heading focuses on movement vector.
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    //Gets the previous location and heading
    Translation2d fieldVelocity=SwerveController.getTranslation2d(swerve.getFieldVelocity());
    //Calculates the desired change in velocity vector - heading is not included.
    Translation2d deltaV = translation.minus(fieldVelocity);

  if (deltaV.getNorm()!=0) { 
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(),
                                            swerve.getPose(),
                                            Constants.Swerve.LOOP_TIME, 
                                            Constants.Swerve.ROBOT_MASS, 
                                            List.of(Constants.Swerve.CHASSIS), 
                                            swerve.getSwerveDriveConfiguration());
    if (Constants.Verbose)
    {SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());
    SmartDashboard.putNumber("AFD Heading", targetHeading.getAngle().getRadians());
    SmartDashboard.putNumber("AFD Heading2", desiredSpeeds.omegaRadiansPerSecond);}

    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, relativeToField.getAsBoolean(), false);  
  } else {
    swerve.drive(translation, targetHeading.getAngle().getRadians(), relativeToField.getAsBoolean(), false); 
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
