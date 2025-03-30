// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import java.util.List;
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
  private final DoubleSupplier vX, vY, heading;
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
   * @param heading DoubleSupplier that supplies the robot's heading angle.
   */
  public AbsoluteFieldDrive(DriveTrain swerve, DoubleSupplier vX, DoubleSupplier vY,
                            DoubleSupplier heading, BooleanSupplier relativeToField)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;
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
    double heading2 = heading.getAsDouble();
    if(Math.abs(heading2)<0.1){
      heading2 = 0;
    }
    ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                                                      new Rotation2d(heading2 * Math.PI));
    Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
    Translation2d fieldVelocity=SwerveController.getTranslation2d(swerve.getFieldVelocity());
    Translation2d deltaV = translation.minus(fieldVelocity);

//    if (deltaV.getNorm()!=0) { 
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(),
                                            swerve.getPose(),
                                            Constants.Swerve.LOOP_TIME, 
                                            Constants.Swerve.ROBOT_MASS, 
                                            List.of(Constants.Swerve.CHASSIS), 
                                            swerve.getSwerveDriveConfiguration());
    if (Constants.Verbose)
    {SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());
    SmartDashboard.putNumber("AFD Heading", heading2 * Math.PI);
    SmartDashboard.putNumber("AFD Heading2", desiredSpeeds.omegaRadiansPerSecond);}

    swerve.drive(translation, heading2 * Math.PI, relativeToField.getAsBoolean(), false);
  
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
