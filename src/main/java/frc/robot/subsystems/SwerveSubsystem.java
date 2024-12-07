package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.DoubleSupplier;

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
    }

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