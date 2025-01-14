package frc.robot.subsystems;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {

    PhotonCamera bigCamera = new PhotonCamera("bigcam");
    PhotonPoseEstimator poseEstimator;
    PhotonPipelineResult latestResult = null;
    Transform3d robotToBigCam = new Transform3d(new Translation3d(7*0.0254, -13*0.0254, 19*0.0254), new Rotation3d(0,0,0));
    Optional<EstimatedRobotPose> estimatedPose;

    double tagID = 0;
    double yaw = 0;
    double pitch = 0;
    double area = 0;

    private StructPublisher <Pose3d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Vision Pose", Pose3d.struct).publish();

    public VisionSubsystem() {
        AprilTagFieldLayout fieldLayout = null;
        try {
            fieldLayout = new AprilTagFieldLayout(AprilTagFields.k2025Reefscape.m_resourceFile);
        } catch (Exception e) {
            System.out.println("This shouldnt have happened");
        }

        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToBigCam);
    }

    public double getID()
    {
        return tagID;
    }

    public EstimatedRobotPose getEstimatedPose() {
        if (latestResult == null) {
            return null;
        }
        Optional<EstimatedRobotPose> pose = poseEstimator.update(latestResult);
        SmartDashboard.putBoolean("EJOIJFEWOIFJW???", pose.isEmpty());
        SmartDashboard.putNumber("timestampritvik???", latestResult.getTimestampSeconds());

        posePublisher.set(pose.isPresent() ? pose.get().estimatedPose : new Pose3d(0, 2, 0, new Rotation3d(0, 0, 0)));
        return pose.isPresent() ? pose.get() : null;
    }

    @Override
    public void periodic() 
    {
        List<PhotonPipelineResult> results = bigCamera.getAllUnreadResults();
        if (!results.isEmpty())
        latestResult = results.get(results.size() - 1);

        SmartDashboard.putBoolean("Camera is connected", bigCamera.isConnected());
        SmartDashboard.putBoolean("Camera has results", !results.isEmpty());

        if (latestResult != null) {
            tagID = latestResult.hasTargets() ? latestResult.getBestTarget().getFiducialId() : -1;
            yaw = latestResult.hasTargets() ? latestResult.getBestTarget().getYaw() : -1;
            pitch = latestResult.hasTargets() ? latestResult.getBestTarget().getPitch() : -1;
            area = latestResult.hasTargets() ? latestResult.getBestTarget().getArea() : -1;
        }

    }
    
}
