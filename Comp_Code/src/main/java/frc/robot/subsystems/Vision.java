// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
//import java.util.HashMap;
import java.util.List;
//import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;
import java.awt.Desktop;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Vision extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  
  /**
   * Photon Vision Simulation
   */
  public              VisionSystemSim     visionSim;
  

  final private boolean visionTroubleShoot = false;
  // This array was designed to aid in defining apriltags useful for targeting.
  //private List<Number> scoringTargets = Arrays.asList(6, 7, 8, 9, 10, 11, 22, 17, 18, 19, 20, 21);
  
  /**
   * Current pose from the pose estimator using wheel odometry.
   */
  private Supplier<Pose2d> currentPose;
  /**
   * Field from {@link swervelib.SwerveDrive#field}
   */
  private Field2d field2d;

  /*
  public enum LevelTarget {
    Level1, Level2, Level3
  }
  BooleanSubscriber rightTarget; 
  BooleanPublisher haveTarget;
  DoublePublisher xTarget;
  DoublePublisher yTarget;
  DoublePublisher thetaTarget;

  //Map<Integer, Pose2d> RtargetLookup = new HashMap<Integer, Pose2d>();
  //Map<Integer, Pose2d> LtargetLookup = new HashMap<Integer, Pose2d>();
  */
  
  /**
     * Constructor for the Vision class.
     *
     * @param currentPose Current pose supplier, should reference {@link SwerveDrive#getPose()}
     * @param field       Current field, should be {@link SwerveDrive#field}
     */
    public Vision(Supplier<Pose2d> currentPose, Field2d field)
    {
      this.currentPose = currentPose;
      this.field2d = field;
      if (Robot.isSimulation())
        {
        visionSim = new VisionSystemSim("Vision");
        visionSim.addAprilTags(fieldLayout);
        if (visionTroubleShoot){
          visionSim.getDebugField();
          
        }

        /*
         * I don't think this is necessary in regular robot, because the cameras are auto detected?
         */
        for (Cameras c : Cameras.values())
        {
            c.addToVisionSim(visionSim);
            if (visionTroubleShoot){
              c.cameraSim.enableDrawWireframe(true);
              CameraServer.addCamera(c.cameraSim.getVideoSimRaw());
              
            }
        }
        
        openSimCameraViews();
        }

       /*The following was removed to address memory issues with the Rio1
        //Red AprilTag Targets
        RtargetLookup.put(6,new Pose2d(541.75,123.67,new Rotation2d(300)));
        RtargetLookup.put(7,new Pose2d(546.87,145.5,new Rotation2d(0)));
        RtargetLookup.put(8,new Pose2d(519.23,180.33,new Rotation2d(60)));
        RtargetLookup.put(9,new Pose2d(486.51,193.33,new Rotation2d(120)));
        RtargetLookup.put(10,new Pose2d(481.39,171.5,new Rotation2d(180)));
        RtargetLookup.put(11,new Pose2d(509.03,136.67,new Rotation2d(240)));
        
        //Blue AprilTag Targets
        RtargetLookup.put(17,new Pose2d(171.65,136.67,new Rotation2d(240)));
        RtargetLookup.put(18,new Pose2d(144.0,171.5,new Rotation2d(180)));
        RtargetLookup.put(19,new Pose2d(149.13,193.33,new Rotation2d(120)));
        RtargetLookup.put(20,new Pose2d(181.84,180.33,new Rotation2d(60)));
        RtargetLookup.put(21,new Pose2d(209.49,145.5,new Rotation2d(0)));
        RtargetLookup.put(22,new Pose2d(204.36,123.67,new Rotation2d(300)));
        
       
        //Red AprilTag Targets
        LtargetLookup.put(6,new Pose2d(519.23,136.67,new Rotation2d(300)));
        LtargetLookup.put(7,new Pose2d(546.87,171.5,new Rotation2d(0)));
        LtargetLookup.put(8,new Pose2d(541.75,193.33,new Rotation2d(60)));
        LtargetLookup.put(9,new Pose2d(509.03,180.33,new Rotation2d(120)));
        LtargetLookup.put(10,new Pose2d(481.39,145.5,new Rotation2d(180)));
        LtargetLookup.put(11,new Pose2d(486.51,123.67,new Rotation2d(240)));
        
        //Blue AprilTag Targets
        LtargetLookup.put(17,new Pose2d(149.13,123.67,new Rotation2d(240)));
        LtargetLookup.put(18,new Pose2d(144.0,145.5,new Rotation2d(180)));
        LtargetLookup.put(19,new Pose2d(171.65,180.33,new Rotation2d(120)));
        LtargetLookup.put(20,new Pose2d(204.36,193.33,new Rotation2d(60)));
        LtargetLookup.put(21,new Pose2d(209.49,171.5,new Rotation2d(0)));
        LtargetLookup.put(22,new Pose2d(181.84,136.67,new Rotation2d(300)));
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("datatable");
        haveTarget = table.getBooleanTopic("haveTarget").publish();
        xTarget = table.getDoubleTopic("xTar").publish();
        yTarget = table.getDoubleTopic("yTar").publish();
        thetaTarget = table.getDoubleTopic("thetaTar").publish();
        haveTarget.set(false);
        xTarget.set(0);
        yTarget.set(0);
        thetaTarget.set(0);

        rightTarget = table.getBooleanTopic("rightTarget").subscribe(false);
*/
  }
  /**
   * Calculates a target pose relative to an AprilTag on the field.
   *
   * @param aprilTag    The ID of the AprilTag.
   * @param robotOffset The offset {@link Transform2d} of the robot to apply to the pose for the robot to position
   *                    itself correctly.
   * @return The target pose of the AprilTag.
   */
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset)
  {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent())
    {
      return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);
    } else
    {
      throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }

  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive} with all of the given poses.
   * Called in swerveDrive periodic
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive)
  {
    if (SwerveDriveTelemetry.isSimulation && swerveDrive.getSimulationDriveTrainPose().isPresent())
    {
      /*
       * In the maple-sim, odometry is simulated using encoder values, accounting for factors like skidding and drifting.
       * As a result, the odometry may not always be 100% accurate.
       * However, the vision system should be able to provide a reasonably accurate pose estimation, even when odometry is incorrect.
       * (This is why teams implement vision system to correct odometry.)
       * Therefore, we must ensure that the actual robot pose is provided in the simulator when updating the vision simulation during the simulation.
       */
      visionSim.update(swerveDrive.getSimulationDriveTrainPose().get());
    }
    //Cycle through each Camera during each click
    for (Cameras camera : Cameras.values())
    {
      //Get a PoseEstimation from the selected Camera - Collected from all tags available
      Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
      if (poseEst.isPresent())
      {
        var pose = poseEst.get();
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(),
                                         pose.timestampSeconds,
                                         camera.curStdDevs);
        //SmartDashboard.putData(field2d.setRobotPose(pose.estimatedPose.toPose2d()));
        
      }
    }

  }

  /**
   * Generates the estimated robot pose. Returns empty if:
   * <ul>
   *  <li> No Pose Estimates could be generated</li>
   * <li> The generated pose estimate was considered not accurate</li>
   * </ul>
   *
   * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Cameras camera)
  {
    Optional<EstimatedRobotPose> poseEst = camera.getEstimatedGlobalPose();
    if (Robot.isSimulation())
    {
      Field2d debugField = visionSim.getDebugField();
      // Uncomment to enable outputting of vision targets in sim.
      poseEst.ifPresentOrElse(
          est ->
              debugField
                  .getObject("VisionEstimation")
                  .setPose(est.estimatedPose.toPose2d()),
          () -> {
            debugField.getObject("VisionEstimation").setPoses();
          });
    } 
    return poseEst;
  }
  /**
   * Get distance of the robot from the AprilTag pose.
   *
   * @param id AprilTag ID
   * @return Distance
   */
  public double getDistanceFromAprilTag(int id)
  {
    Optional<Pose3d> tag = fieldLayout.getTagPose(id);
    return tag.map(pose3d -> PhotonUtils.getDistanceToPose(currentPose.get(), pose3d.toPose2d())).orElse(-1.0);
  }

  /**
   * Get tracked target from a camera of AprilTagID
   *
   * @param id     AprilTag ID
   * @param camera Camera to check.
   * @return Tracked target.
   */
  public PhotonTrackedTarget getTargetFromId(int id, Cameras camera)
  {
    PhotonTrackedTarget target = null;
    for (PhotonPipelineResult result : camera.resultsList)
    {
      if (result.hasTargets())
      {
        for (PhotonTrackedTarget i : result.getTargets())
        {
          if (i.getFiducialId() == id)
          {
            return i;
          }
        }
      }
    }
    return target;

  }

  /**
   * Vision simulation.
   *
   * @return Vision Simulation
   */
  public VisionSystemSim getVisionSim()
  {
    return visionSim;
  }
/**
   * Open up the photon vision camera streams on the localhost, assumes running photon vision on localhost.
   */
  private void openSimCameraViews()
  {
    if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE))
    {
      try
      {
        Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
        Desktop.getDesktop().browse(new URI("http://localhost:1184/"));
        Desktop.getDesktop().browse(new URI("http://localhost:1186/"));
      } catch (IOException | URISyntaxException e)
      {
        e.printStackTrace();
      }
    }
  }


  enum Cameras
  {
    FRONT_CAM("Arducam_OV9782_USB_Camera1",
              new Rotation3d(0,0,0),
              new Translation3d(Units.inchesToMeters(2),
                                Units.inchesToMeters(18),
                                Units.inchesToMeters(6.5)),
              VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1)),
    REAR_CAM("Arducam_OV9782_USB_Camera2",
              new Rotation3d(0,0,Math.PI),
              new Translation3d(Units.inchesToMeters(-5),
                                Units.inchesToMeters(22),
                                Units.inchesToMeters(0)),
              VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));
              
    /*
     ======================
     These are Camera level variables to be assigned to each camera
     ===========================
     */
              /**
     * Latency alert to use when high latency is detected.
     */
    public final  Alert                        latencyAlert;
    /**
     * Camera instance for comms.
     */
    public final  PhotonCamera                 camera;
    /**
     * Pose estimator for camera.
     */
    public final  PhotonPoseEstimator          poseEstimator;
    /**
     * Standard Deviation for single tag readings for pose estimation.
     */
    private final Matrix<N3, N1>               singleTagStdDevs;
    /**
     * Standard deviation for multi-tag readings for pose estimation.
     */
    private final Matrix<N3, N1>               multiTagStdDevs;
    /**
     * Transform of the camera rotation and translation relative to the center of the robot
     */
    private final Transform3d                  robotToCamTransform;
    /**
     * Current standard deviations used.
     */
    public        Matrix<N3, N1>               curStdDevs;
    /**
     * Estimated robot pose.
     */
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    /**
     * Simulated camera instance which only exists during simulations.
     */
    public        PhotonCameraSim              cameraSim;
    /**
     * Results list to be updated periodically and cached to avoid unnecessary queries.
     */
    public        List<PhotonPipelineResult>   resultsList       = new ArrayList<>();
    /**
     * Last read from the camera timestamp to prevent lag due to slow data fetches.
     */
    private       double                       lastReadTimestamp = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);

    /**
     * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
     * estimation noise on an actual robot.
     *
     * @param name                  Name of the PhotonVision camera found in the PV UI.
     * @param robotToCamRotation    {@link Rotation3d} of the camera.
     * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
     * @param singleTagStdDevs      Single AprilTag standard deviations of estimated poses from the camera.
     * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
     */
    Cameras(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation,
            Matrix<N3, N1> singleTagStdDevs, Matrix<N3, N1> multiTagStdDevsMatrix)
    {
      latencyAlert = new Alert("'" + name + "' Camera is experiencing high latency.", AlertType.kWarning);

      camera = new PhotonCamera(name);

      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);

      poseEstimator = new PhotonPoseEstimator(Vision.fieldLayout,
                                              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                                              robotToCamTransform);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      this.singleTagStdDevs = singleTagStdDevs;
      this.multiTagStdDevs = multiTagStdDevsMatrix;
      /*
       * =======================
       * This bit of code is used if the actual Camera information is not available (i.e. robot in sim mode)
       * =======================
       */
      if (Robot.isSimulation())
      {
        SimCameraProperties cameraProp = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraProp.setFPS(30);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
      }
    }

    /**
     * Add camera to {@link VisionSystemSim} for simulated photon vision.
     *
     * @param systemSim {@link VisionSystemSim} to use.
     */
    public void addToVisionSim(VisionSystemSim systemSim)
    {
      if (Robot.isSimulation())
      {
        systemSim.addCamera(cameraSim, robotToCamTransform);
      }
    }
    /**
     * Get the result with the least ambiguity from the best tracked target within the Cache. This may not be the most
     * recent result!
     *
     * @return The result in the cache with the least ambiguous best tracked target. This is not the most recent result!
     */
    public Optional<PhotonPipelineResult> getBestResult()
    {
      if (resultsList.isEmpty())
      {
        return Optional.empty();
      }

      PhotonPipelineResult bestResult       = resultsList.get(0);
      double               amiguity         = bestResult.getBestTarget().getPoseAmbiguity();
      double               currentAmbiguity = 0;
      for (PhotonPipelineResult result : resultsList)
      {
        currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
        if (currentAmbiguity < amiguity && currentAmbiguity > 0)
        {
          bestResult = result;
          amiguity = currentAmbiguity;
        }
      }
      return Optional.of(bestResult);
    }

    /**
     * Get the latest result from the current cache.
     *
     * @return Empty optional if nothing is found. Latest result if something is there.
     */
    public Optional<PhotonPipelineResult> getLatestResult()
    {
      return resultsList.isEmpty() ? Optional.empty() : Optional.of(resultsList.get(0));
    }

    /**
     * Get the estimated robot pose. Updates the current robot pose estimation, standard deviations, and flushes the
     * cache of results.
     *
     * @return Estimated pose.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose()
    {
      updateUnreadResults();
      return estimatedRobotPose;
    }

    /**
     * Update the latest results, cached with a maximum refresh rate of 1req/15ms. Sorts the list by timestamp.
     */
    private void updateUnreadResults()
    {
      double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();
      double currentTimestamp    = Microseconds.of(NetworkTablesJNI.now()).in(Seconds);
      double debounceTime        = Milliseconds.of(15).in(Seconds);
      for (PhotonPipelineResult result : resultsList)
      {
        mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
      }
      if ((resultsList.isEmpty() || (currentTimestamp - mostRecentTimestamp >= debounceTime)) &&
          (currentTimestamp - lastReadTimestamp) >= debounceTime)
      {
        resultsList = Robot.isReal() ? camera.getAllUnreadResults() : cameraSim.getCamera().getAllUnreadResults();
        lastReadTimestamp = currentTimestamp;
        //System.err.print("Result info: ");
        //System.err.println(resultsList.size());
        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
          return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        });
        if (!resultsList.isEmpty())
        {
          updateEstimatedGlobalPose();
        }
      }
    }
    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should only be called once
     * per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link Cameras#updateEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets used for
     * estimation.
     */
    private void updateEstimatedGlobalPose()
    {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : resultsList)
      {
        visionEst = poseEstimator.update(change);
        updateEstimationStdDevs(visionEst, change.getTargets());
      }
      estimatedRobotPose = visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard deviations based
     * on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     */
    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets)
    {
      if (estimatedPose.isEmpty())
      {
        // No pose input. Default to single-tag std devs
        curStdDevs = singleTagStdDevs;

      } else
      {
        // Pose present. Start running Heuristic
        var    estStdDevs = singleTagStdDevs;
        int    numTags    = 0;
        double avgDist    = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets)
        {
          var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
          if (tagPose.isEmpty())
          {
            continue;
          }
          numTags++;
          avgDist +=
              tagPose
                  .get()
                  .toPose2d()
                  .getTranslation()
                  .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0)
        {
          // No tags visible. Default to single-tag std devs
          curStdDevs = singleTagStdDevs;
        } else
        {
          // One or more tags visible, run the full heuristic.
          avgDist /= numTags;
          // Decrease std devs if multiple targets are visible
          if (numTags > 1)
          {
            estStdDevs = multiTagStdDevs;
          }
          // Increase std devs based on (average) distance
          if (numTags == 1 && avgDist > 4)
          {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
          } else
          {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          }
          curStdDevs = estStdDevs;
        }
      }
    }
  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
 
  /**
   * This method is designed to return a Pose2d for the desired Scoring Position
   * based on current position and selected targets.
   */
/*
   public void getTargetPose(){
    Pose2d target;
    for (PhotonPipelineResult result : Cameras.FRONT_CAM.resultsList)
    {
      if (result.hasTargets())
      {
        if (scoringTargets.contains(result.getBestTarget().getFiducialId()))
          {
            haveTarget.set(true);
            int targetID = result.getBestTarget().getFiducialId();
            if (rightTarget.get()) {
              target = RtargetLookup.get(targetID);
            } else {
              target = LtargetLookup.get(targetID);
            }
            xTarget.set(target.getX());
            yTarget.set(target.getY());
            thetaTarget.set(target.getRotation().getRadians());
            } 
          } 
        }
      }
*/



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //getTargetPose();
        
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    visionSim.update(currentPose.get());
  }

  
}
