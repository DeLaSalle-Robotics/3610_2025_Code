import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.io.File;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.DriveTrain;

public class TestVision {
    Vision m_vision;
    DriveTrain      m_driveTrain;
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
        System.out.print("Tag PoseX:");
        Pose2d testPose = m_vision.getAprilTagPose(18, cam_trans);
        System.out.println(testPose.getX());
        assertTrue(testPose.getX() < 10);
    }

    @Test
    void getPose18y() {
        System.out.print("Tag PoseY:");
        Pose2d testPose = m_vision.getAprilTagPose(18, cam_trans);
        System.out.println(testPose.getY());
        assertTrue(testPose.getY() < 10);
    }

    @Test
    void testPoseEstimateY(){
        System.out.print("PoseEstimateY:");
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
        System.out.print("PoseEstimateX:");
        try {
            Pose2d testPose = m_vision.getPoseEstimation().get();
            System.out.println(testPose.getX());
            assertTrue((testPose.getX() > 0.9) && (testPose.getX() < 1.1));
        } catch (Exception e) {
            System.out.print(e);
        }
    }

    //@Test
    void testDrivePoseX(){
        m_driveTrain = new DriveTrain(new File("src/main/deploy/swerve"));
        System.out.print("Drive PoseX:");
        
        Pose2d testPose = m_driveTrain.getPose();
        System.out.println(testPose.getX());
        m_driveTrain.close();
        assertTrue((testPose.getX() > 0.9) && (testPose.getX() < 1.1));
    }

    @Test
    void testSetPoseX(){
        m_driveTrain = new DriveTrain(new File("src/main/deploy/swerve"));
        Pose2d testPose = m_vision.getAprilTagPose(18, cam_trans);
        m_driveTrain.resetPose(testPose);
        Pose2d newPose = m_driveTrain.getPose();
        System.out.print("New Pose X:");
        System.out.println(newPose.getX());
        System.out.print("New Pose Y:");
        System.out.println(newPose.getY());
        assertTrue((testPose.getX() > 3.5) && (testPose.getX() < 3.7));
        
        
    }
}

