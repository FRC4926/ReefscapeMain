package frc.robot.subsystems;
import java.io.IOException;
import java.util.List;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonUtils;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import frc.robot.RobotContainer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {
    PhotonCamera bigCamera;
    PhotonCameraSim bigCameraSim;
    PhotonPipelineResult latestResult = null;

    VisionSystemSim visionSim = new VisionSystemSim("main");
    TargetModel myModel = new TargetModel(10, 10);
    VisionTargetSim targetSim = new VisionTargetSim(new Pose3d(1, 2, 3, new Rotation3d(0, 0, Math.PI)), myModel);

    double tagID = 0;
    double yaw = 0;
    double pitch = 0;
    double area = 0;

    public VisionSubsystem() {
        bigCamera = new PhotonCamera("bigcam");

        visionSim.addVisionTargets(targetSim);
        try {
            visionSim.addAprilTags(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile));
        } catch (Exception e) {
            System.out.println("This should NOT happen."); 
        }

        SimCameraProperties camProps = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        camProps.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        camProps.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        camProps.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        camProps.setAvgLatencyMs(35);
        camProps.setLatencyStdDevMs(5);

        bigCameraSim = new PhotonCameraSim(bigCamera, camProps);
        visionSim.addCamera(bigCameraSim, new Transform3d(new Translation3d(0.1, 0, 0.5), new Rotation3d(0, Math.toRadians(-15), 0)));
    }

    public double getID()
    {
        return tagID;
    }

    public double getYaw()
    {
        return yaw;
    }


    @Override
    public void periodic() 
    {
        visionSim.update(RobotContainer.drivetrain.getState().Pose);
        visionSim.getDebugField();

        List<PhotonPipelineResult> results = bigCamera.getAllUnreadResults();
        if (!results.isEmpty())
        latestResult = results.get(results.size() - 1);

        SmartDashboard.putBoolean("Camera is connected", bigCamera.isConnected());
        SmartDashboard.putBoolean("Camera has results", !results.isEmpty());

        if (latestResult != null) {
            var bestTarget = latestResult.getBestTarget();
            tagID = latestResult.hasTargets() ? latestResult.getBestTarget().getFiducialId() : -1;
            yaw = latestResult.hasTargets() ? latestResult.getBestTarget().getYaw() : -1;
            pitch = latestResult.hasTargets() ? latestResult.getBestTarget().getPitch() : -1;
            area = latestResult.hasTargets() ? latestResult.getBestTarget().getArea() : -1;

            try {
                AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
                Pose3d tagPose = fieldLayout.getTagPose((int) tagID).orElse(null);
    
                if (tagPose != null) {
                    // Calculate robot pose using the tag pose and camera-to-tag transform
                    Transform3d cameraToTag = bestTarget.getBestCameraToTarget();
                    Pose3d robotPose = tagPose.transformBy(cameraToTag.inverse());
    
                    // Log the estimated pose
                    SmartDashboard.putNumber("Estimated X", robotPose.getX());
                    SmartDashboard.putNumber("Estimated Y", robotPose.getY());
                    SmartDashboard.putNumber("Estimated Z", robotPose.getZ());
                    SmartDashboard.putString("Robot Pose", robotPose.toString());
                }
            } catch (IOException e) {
                SmartDashboard.putString("Vision Error", "Failed to load field layout: " + e.getMessage());
            }
        }

    }
}
