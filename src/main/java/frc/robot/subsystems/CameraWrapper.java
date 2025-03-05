package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;


import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import frc.robot.Robot;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraWrapper {
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator poseEstimator;
    private final StructPublisher<Pose2d> posePublisher;
    private Transform3d robotToCam;
    private PhotonPipelineResult latestResult;
    private boolean publishPose;

    public CameraWrapper(String camName, Transform3d _robotToCam, AprilTagFieldLayout fieldLayout, boolean _publishPose) {
        camera = new PhotonCamera(camName);

        robotToCam = _robotToCam;
        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam.inverse());
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        publishPose = _publishPose;
        if (publishPose) {
            posePublisher = NetworkTableInstance.getDefault().getStructTopic(camName + " pose", Pose2d.struct).publish();
        } else {
            posePublisher = null;
        }
   
        if (Robot.isSimulation()) {
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

            cameraSim = new PhotonCameraSim(camera, camProps);
        } else {
            cameraSim = null;
        }
    }

    public void addToSimulator(VisionSystemSim sim) {
        if (Robot.isSimulation()) {
            sim.addCamera(cameraSim, robotToCam);
        }
    }

    public Transform3d getRobotToCam() {
        return robotToCam;
    }

    public boolean isConnected() {
        return camera.isConnected();
    }

    public void checkForResult() {
        if (!camera.isConnected())
            return;

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (!results.isEmpty())
            latestResult = results.get(results.size() - 1);
    }

    public PhotonPipelineResult getLatestResult() {
        return latestResult;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (!camera.isConnected() || latestResult == null || !latestResult.hasTargets())
        {
            if (publishPose) {
                posePublisher.set(new Pose2d());
            }
            return Optional.empty();
        } 

        List<PhotonTrackedTarget> betterTargets = new ArrayList<>();
        for (PhotonTrackedTarget target : latestResult.targets)
        {
            if (target.getPoseAmbiguity() <= 0.2)
            {
                betterTargets.add(target);
            }
        }
        PhotonPipelineResult betterResult = new PhotonPipelineResult(latestResult.metadata.sequenceID, latestResult.metadata.captureTimestampMicros, latestResult.metadata.publishTimestampMicros, latestResult.metadata.timeSinceLastPong, betterTargets);

        var estimated = poseEstimator.update(betterResult);
        SmartDashboard.putNumber(camera.getName() + " target count", betterResult.targets.size());


        if (estimated.isEmpty()) return Optional.empty();

        if (publishPose) {
            if (estimated.isPresent()) {
                posePublisher.set(estimated.get().estimatedPose.toPose2d());
            } else {
                posePublisher.set(new Pose2d());
            }
        }

        //SmartDashboard.putBoolean("Pose estimator is present for" + camera.getName(), estimated.isPresent());

        return estimated;

    }

    public double getID()
    {
        if (latestResult != null)
        {
            var bestTarget = latestResult.getBestTarget();
            return latestResult.hasTargets() ? bestTarget.getFiducialId() : -1;
        } else
        {
            return -1;
        }
    }

    public PhotonTrackedTarget getBestTarget()
    {
        return latestResult.getBestTarget();
    }

    public PhotonCamera getCamera()
    {
        return camera;
    }
}