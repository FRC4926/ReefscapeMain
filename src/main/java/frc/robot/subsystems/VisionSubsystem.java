package frc.robot.subsystems;
import java.io.IOException;
import java.util.List;
import java.util.Optional;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class VisionSubsystem extends SubsystemBase {
    PhotonCamera bigCamera;
    PhotonCameraSim bigCameraSim;
    PhotonPipelineResult latestResult = null;

    VisionSystemSim visionSim = new VisionSystemSim("main");
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

    double tagID = 0;
    public Pose3d robotPose = new Pose3d();

    Pose3d tagPose = new Pose3d(); 
    double yaw = 0;
    double pitch = 0;
    double area = 0;
    AprilTagFieldLayout fieldLayout = null;
    Transform3d robotToBigCam = new Transform3d(new Translation3d(7*0.0254, -13*0.0254, 19*0.0254), new Rotation3d(0,-0.262,0));

    Optional<EstimatedRobotPose> estimatedPose;
    PhotonPoseEstimator poseEstimator;
    private Matrix<N3, N1> curStdDevs;
    AprilTagFieldLayout aprilTagFieldLayout;
    public VisionSubsystem() {
        bigCamera = new PhotonCamera("bigcam");

       // visionSim.addVisionTargets(targetSim);
        try
        {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
            visionSim.addAprilTags(fieldLayout);

        } catch (Exception e)
        {
            SmartDashboard.putString("bro actually errored....", e.toString());
        }
        



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

        bigCameraSim = new PhotonCameraSim(bigCamera, camProps);
        visionSim.addCamera(bigCameraSim, new Transform3d(new Translation3d(7*0.0254, -13*0.0254, 19*0.0254), new Rotation3d(0, Math.toRadians(-15), 0)));
        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBigCam.inverse());
        //poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public double getID()
    {
        return tagID;
    }

    public double getYaw()
    {
        return yaw;
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
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (latestResult != null && latestResult.hasTargets()) {
            var ret = poseEstimator.update(latestResult);
            SmartDashboard.putBoolean("pose estimator is present", ret.isPresent());
            return ret;

        } else {
            return Optional.empty();
        }
    }

    @Override
    public void periodic() 
    {
       visionSim.update(RobotContainer.drivetrain.getState().Pose);
        //visionSim.getDebugField();
    
        List<PhotonPipelineResult> results = bigCamera.getAllUnreadResults();
        if (!results.isEmpty())
            latestResult = results.get(results.size() - 1);
    
        SmartDashboard.putBoolean("Camera is connected", bigCamera.isConnected());
        SmartDashboard.putBoolean("Camera has results", !results.isEmpty());
        SmartDashboard.putBoolean("Result has targets", latestResult.hasTargets());
       // Optional<EstimatedRobotPose> poses = getEstimatedGlobalPose();
        // if (poses.isPresent()) {
        //     Pose3d tagPose = poses.get().estimatedPose;
        //     Transform3d cameraToTag = latestResult.getBestTarget().getBestCameraToTarget();
        //     robotPose = tagPose.transformBy(cameraToTag.inverse());
        // }
        // posePublisher.set(robotPose);

    
       
       // Pose3d robotPose = new Pose3d(); // Field-relative robot pose
    
        //RobotContainer.drivetrain.addVisionMeasurement(tagPose.toPose2d(), area);
       
    
        if (latestResult != null) {
            var bestTarget = latestResult.getBestTarget();
            tagID = latestResult.hasTargets() ? bestTarget.getFiducialId() : -1;
            yaw = latestResult.hasTargets() ? bestTarget.getYaw() : -1;
            pitch = latestResult.hasTargets() ? bestTarget.getPitch() : -1;
            area = latestResult.hasTargets() ? bestTarget.getArea() : -1;
    
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

}
