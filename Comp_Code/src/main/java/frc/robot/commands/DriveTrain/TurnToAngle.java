package frc.robot.commands.DriveTrain;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends Command{
    private final DriveTrain swerve;
    private Pose2d finalPose;
    private double error;
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
    public TurnToAngle(DriveTrain swerve, Pose2d finalPose) {
        this.swerve = swerve;
        this.finalPose = finalPose;
        
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        double currentAngle = swerve.getPose().getRotation().getRadians();
        double targetAngle = finalPose.getRotation().getRadians();
        double error = targetAngle - currentAngle;
        double rotationAmp = 0.2 * error;
        swerve.drive(new Translation2d(0,0), rotationAmp, true);
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return (Math.abs(error) < 0.08);
    }

    }
