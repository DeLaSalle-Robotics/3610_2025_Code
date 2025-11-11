import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Vision;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class TestVision {
    Vision m_vision;
    SwerveDrive         swerveDrive;
    Transform2d   cam_trans;
    Pose2d roboPose;
    Field2d field2d;
    
    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
        field2d = new Field2d();
        
        roboPose = new Pose2d(new Translation2d(Meter.of(1),
                                Meter.of(3)),
                                Rotation2d.fromDegrees(0));

        m_vision = new Vision(() -> roboPose, field2d);
        cam_trans = new Transform2d(0,0, new Rotation2d(0));
    }

    @Test
    void getPose18x() {
        Pose2d testPose = m_vision.getAprilTagPose(18, cam_trans);
        //System.out.println(testPose.getX());
        assertTrue(testPose.getX() < 10);
    }

    @Test
    void getPose18y() {
        Pose2d testPose = m_vision.getAprilTagPose(18, cam_trans);
        //System.out.println(testPose.getY());
        assertTrue(testPose.getY() < 10);
    }

    @Test
    void testPoseEstimateY(){
        try {
            Pose2d testPose = m_vision.getPoseEstimation().get();
            System.out.println(testPose.getY());
            assertTrue((testPose.getY() > 2.9) && (testPose.getY() < 3.1));
        } catch (Exception e) {
            System.out.print(e);
        }
        
    }

    @Test
    void testPoseEstimateX(){
        try {
            Pose2d testPose = m_vision.getPoseEstimation().get();
            System.out.println(testPose.getX());
            assertTrue((testPose.getX() > 0.9) && (testPose.getX() < 1.1));
        } catch (Exception e) {
            System.out.print(e);
        }
    }
}

