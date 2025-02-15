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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveToPoint extends Command{
    private final DriveTrain swerve;
    private Pose2d finalPose;
    private boolean initRotation = false;
    private PathPlannerPath path;
    private GoalEndState endGoalState;
    private static FlippingUtil flipUtil;
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
    public DriveToPoint(DriveTrain swerve, Pose2d finalPose) {
        this.swerve = swerve;
        this.finalPose = finalPose;
        flipUtil = new FlippingUtil();
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
        endGoalState = new GoalEndState(0.0, finalPose.getRotation());
        path = Pathfinding.getCurrentPath(constraints, endGoalState);
        if (swerve.isRedAlliance()) {
            finalPose = flipUtil.flipFieldPose(finalPose);
        }
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    }
