package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;

public class SwerveSubsystem extends SubsystemBase{

    /**
   * Swerve drive object.
   */
    private final SwerveDrive swerveDrive;
 
    public SwerveSubsystem(File directory){
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try
        {
        //This will create a swerveDrive object with a potision 1 m infront of the drivers station and 4 m from the right wall.    
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.Swerve.MAX_SPEED,
                                                                    new Pose2d(new Translation2d(Meter.of(1),
                                                                                                Meter.of(4)),
                                                                                Rotation2d.fromDegrees(0)));
       } catch (Exception e)
        {
        throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        //!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        if (SwerveDriveTelemetry.isSimulation) {swerveDrive.setCosineCompensator(false);}
        swerveDrive.setAngularVelocityCompensation(true,
                                                true,
                                                0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                    1); 
        // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. 
        // Throws warning if not possible
        swerveDrive.pushOffsetsToEncoders(); 
        setupPathPlanner();
    }
    //METHODS
    //========================================
    @Override
    public void periodic(){}

    //Path Planning Methods
    //----------------------------------------
    public void setupPathPlanner(){
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
            final boolean enableFeedforward = true;
            AutoBuilder.configure(
            this::getPose,
            // Robot pose supplier
            this::resetOdometry,
            // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotVelocity,
            // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speedsRobotRelative, moduleFeedForwards) -> {
                if (enableFeedforward)
                {
                swerveDrive.drive(
                    speedsRobotRelative,
                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                    moduleFeedForwards.linearForces()
                                );
                } else
                {
                swerveDrive.setChassisSpeeds(speedsRobotRelative);
                }
            },
            // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController(
                // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PIDConstants(5.0, 0.0, 0.0),
                // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0)
                // Rotation PID constants
            ),
            config,
            // The robot configuration
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent())
                {
                return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
            // Reference to this subsystem to set requirements
                            );
        } catch (Exception e)
        {
          // Handle exception as needed
          e.printStackTrace();
        }
    }
    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialHolonomicPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }
    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose()
    {
        return swerveDrive.getPose();
    }
    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity()
    {
        return swerveDrive.getRobotVelocity();
    }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory)
    {
        swerveDrive.postTrajectory(trajectory);
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro()
    {
        swerveDrive.zeroGyro();
    }

      /**
     * Checks if the alliance is red, defaults to false if alliance isn't available.
     *
     * @return true if the red alliance, false if blue. Defaults to false if none is available.
     */
    private boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    /**
     * This will zero (calibrate) the robot to assume the current position is facing forward
     * <p>
     * If red alliance rotate the robot 180 after the drviebase zero command
     */
    public void zeroGyroWithAlliance()
    {
        if (isRedAlliance())
        {
        zeroGyro();
        //Set the pose 180 degrees
        resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else
        {
        zeroGyro();
        }
    }


    //COMMANDS
    //========================================
    
    /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @param triggerValue     Allows for realtime control of field oriented driving
   * @return Drive command.
   */
  
    public Command driveCommand(DoubleSupplier translationX, 
                                DoubleSupplier translationY, 
                                DoubleSupplier angularRotationX, 
                                DoubleSupplier triggerValue){
        return run(() -> {
          // Make the robot move
          swerveDrive.drive(
            new Translation2d(
              Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
              Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()
            ),
            Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
            (triggerValue.getAsDouble() < 0.5),
            false
            );
          }
        );
      
    
}

}