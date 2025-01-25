package frc.robot.subsystems;
import java.io.IOException;
import java.lang.StackWalker.Option;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.lang.model.util.ElementScanner14;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
   private List<CameraWrapper> camWrappers = new ArrayList<>();
    // PhotonCamera frontCam;
    // PhotonCamera backCam;
    //  PhotonCameraSim bigCameraSim;
    // PhotonPipelineResult latestResultFront = null;
    // PhotonPipelineResult latestResultBack = null;


    //VisionSystemSim visionSim = new VisionSystemSim("main");
    // TargetModel myModel = new TargetModel(10, 10);
    // VisionTargetSim targetSim = new VisionTargetSim(new Pose3d(1, 2, 3, new Rotation3d(0, 0, Math.PI)), myModel);
    private final StructPublisher<Pose3d> posePublisher =
            NetworkTableInstance.getDefault()
                    .getStructTopic("sheesh", Pose3d.struct)
                    .publish();
                    private final StructPublisher<Pose3d> posePublisherBruh =
                    NetworkTableInstance.getDefault()
                            .getStructTopic("bruh", Pose3d.struct)
                            .publish();

    double tagIDFront = 0;
    public Pose3d robotPose = new Pose3d();

    Pose3d tagPose = new Pose3d(); 
    double yawFront = 0;
    double pitchFront = 0;
    double areaFront = 0;
    AprilTagFieldLayout fieldLayout = null;
    // Transform3d robotToFrontCam = new Transform3d(new Translation3d(0*0.0254, -10.5*0.0254, 12.5*0.0254), new Rotation3d(0,0,Math.PI));
    // Transform3d robotToBackCam = new Transform3d(new Translation3d(9.5*0.0254, -10*0.0254, 18*0.0254), new Rotation3d(0,0,Math.PI/2));

    Optional<EstimatedRobotPose> estimatedPose;
    // PhotonPoseEstimator poseEstimatorFront;
    // PhotonPoseEstimator poseEstimatorBack;
    private Matrix<N3, N1> curStdDevs;
    AprilTagFieldLayout aprilTagFieldLayout;
    public VisionSubsystem() {
       // visionSim.addVisionTargets(targetSim);
        try
        {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
            //visionSim.addAprilTags(fieldLayout);

        } catch (Exception e)
        {
            SmartDashboard.putString("bro actually errored....", e.toString());
        }
        updateOrigin();
        addCamera("ArducamFront", new Transform3d(new Translation3d(0*0.0254, -10.5*0.0254, 12.5*0.0254), new Rotation3d(0,0,Math.PI)), fieldLayout);
        addCamera("ArducamBack", new Transform3d(new Translation3d(9.5*0.0254, -10*0.0254, 18*0.0254), new Rotation3d(0,0,Math.PI/2)), fieldLayout);
        addCamera("ArducamRight", new Transform3d(new Translation3d(-8.25*0.0254, -4*0.0254, 13.25*0.0254), new Rotation3d(0,0,3*Math.PI/2)), fieldLayout);
        addCamera("ArducamLeft", new Transform3d(new Translation3d(-8.5*0.0254, 11.5*0.0254, 10*0.0254), new Rotation3d(0,0,0)), fieldLayout);
        //addCamera("limelight", new Transform3d(new Translation3d(-7.5*0.0254, -4*0.0254, 13.5*0.0254), new Rotation3d(0,0,-Math.PI/2)), fieldLayout);
        



        SimCameraProperties camProps = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        camProps.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        camProps.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        camProps.setFPS(45);
        // The average and standard deviation in milliseconds of image data latency.
        camProps.setAvgLatencyMs(25);
        camProps.setLatencyStdDevMs(5);

        //bigCameraSim = new PhotonCameraSim(bigCamera, camProps);
       // visionSim.addCamera(bigCameraSim, new Transform3d(new Translation3d(7*0.0254, -13*0.0254, 19*0.0254), new Rotation3d(0, Math.toRadians(-15), 0)));
        // poseEstimatorFront = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToFrontCam.inverse());
        // poseEstimatorBack = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBackCam.inverse());

        //poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void updateOrigin()
    {
        if (DriverStation.getAlliance().get().toString().equals("Red"))
        {
            fieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        } else if (DriverStation.getAlliance().get().toString().equals("Blue"))
        {
            fieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        } else
        {
            System.out.println("Bro actually errored");
        }
    }
    public void addCamera(String camName, Transform3d robotToCam, AprilTagFieldLayout _fieldLayout) {
        camWrappers.add(new CameraWrapper(camName, robotToCam, _fieldLayout));
    }

    public List<CameraWrapper> getCameras()
    {
        return camWrappers;
    }

    public double getIDFront()
    {
        return tagIDFront;
    }

    public double getYawFront()
    {
        return yawFront;
    }
    public Pose3d getRobotPose() {
        return robotPose;
    }
    public Pose3d getTagPose() {
        // Translation3d offsetFromTag = new Translation3d(1.0, tagPose.getRotation());
        // return tagPose.transformBy(new Transform3d(offsetFromTag, new Rotation3d()));
        Translation2d offsetFromTag = new Translation2d(-0.4, tagPose.getRotation().toRotation2d().rotateBy(Rotation2d.fromDegrees(-60)));

        return tagPose.transformBy(new Transform3d(offsetFromTag.getX(), offsetFromTag.getY(), 0, new Rotation3d()));

        // Pose3d sheesh = new Pose3d(tagPose.getX() + Math.cos(tagPose.getRotation().getAngle()), tagPose.getY() + Math.sin(tagPose.getRotation().getAngle()), tagPose.getZ(), tagPose.getRotation());
     
        // return sheesh;
    }
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPoseFront() {
    //     if (latestResultFront != null && latestResultFront.hasTargets()) {
    //         var estimated = poseEstimatorFront.update(latestResultFront);
    //         SmartDashboard.putBoolean("pose estimator is present front", estimated.isPresent());
    //         if (estimated.isEmpty()) return Optional.empty();

    //         return estimated;

    //     } else {
    //         return Optional.empty();
    //     }
    // }

    // public Optional<EstimatedRobotPose> getEstimatedGlobalPoseBack() {
    //     if (latestResultBack != null && latestResultBack.hasTargets()) {
    //         var estimated = poseEstimatorBack.update(latestResultBack);
    //         SmartDashboard.putBoolean("pose estimator is present back", estimated.isPresent());
    //         if (estimated.isEmpty()) return Optional.empty();

    //         // Transform3d cameraToTag = latestResult.getBestTarget().getBestCameraToTarget();
    //         // var ret = estimated.get().estimatedPose.transformBy(cameraToTag.inverse());

    //         // return Optional.of(new EstimatedRobotPose(ret, estimated.get().timestampSeconds, null, null));

    //         return estimated;

    //     } else {
    //         return Optional.empty();
    //     }
    // }

    // public List<Optional<EstimatedRobotPose>> getEstimatedGlobalPoses() {
    //     List<Optional<EstimatedRobotPose>> ret = new ArrayList<>(camWrappers.size());
    //     for (CameraWrapper camWrapper : camWrappers) {
    //         ret.add(camWrapper.getEstimatedGlobalPose());
    //     }

    //     return ret;
    // }


    @Override
    public void periodic() 
    {
        for (CameraWrapper cam: camWrappers)
        {
            cam.checkForResult();
        }
        SmartDashboard.putString("Alliance", DriverStation.getAlliance().get().toString());
       //visionSim.update(RobotContainer.drivetrain.getState().Pose);
        //visionSim.getDebugField();
    

       // Optional<EstimatedRobotPose> poses = getEstimatedGlobalPose();
        // if (poses.isPresent()) {
        //     Pose3d tagPose = poses.get().estimatedPose;
        //     Transform3d cameraToTag = latestResult.getBestTarget().getBestCameraToTarget();
        //     robotPose = tagPose.transformBy(cameraToTag.inverse());
        // }
        // posePublisher.set(robotPose);

    
       
       // Pose3d robotPose = new Pose3d(); // Field-relative robot pose
    
        //RobotContainer.drivetrain.addVisionMeasurement(tagPose.toPose2d(), area);
       
    

    
            // try {
            //     // Load AprilTag field layout and get the tag's pose
            //     AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
            //     tagPose = fieldLayout.getTagPose((int) tagID).orElse(null);
    
            //     if (tagPose != null) {
            //         // Calculate robot pose using the tag pose and camera-to-tag transform
            //         Transform3d cameraToTag = bestTarget.getBestCameraToTarget();
            //         robotPose = tagPose.transformBy(cameraToTag.inverse());
    
            //         // Log the estimated robot pose
            //         SmartDashboard.putNumber("tagID", tagID);
            //         SmartDashboard.putNumber("Robot Y", robotPose.getY());
            //         SmartDashboard.putNumber("Robot Z", robotPose.getZ());
            //         SmartDashboard.putString("Robot Pose", robotPose.toString());
            //         posePublisher.set(getTagPose());
            //         posePublisherBruh.set(robotPose);
            //     }
            // } catch (IOException e) {
            //     SmartDashboard.putString("Vision Error", "Failed to load field layout: " + e.getMessage());
            // }
        }
    
        // Publish the robot pose to AdvantageScope
        
    }
