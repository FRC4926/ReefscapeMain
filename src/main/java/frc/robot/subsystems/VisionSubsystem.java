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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {

    PhotonCamera bigCamera = new PhotonCamera("bigcam");
    PhotonPipelineResult latestResult = null;

    Transform3d robotToBigCam = new Transform3d(new Translation3d(7*0.0254, -13*0.0254, 19*0.0254), new Rotation3d(0,0,0));
    Optional<EstimatedRobotPose> estimatedPose;
    AprilTagFieldLayout fieldLayout = null;
    PhotonPoseEstimator poseEstimator;
    private Matrix<N3, N1> curStdDevs;

    Pose3d robotPose = null;

    double tagID = 0;
    double yaw = 0;
    double pitch = 0;
    double area = 0;
    
    AprilTagFieldLayout aprilTagFieldLayout;
    private StructPublisher <Pose3d> posePublisher = NetworkTableInstance.getDefault().getStructTopic("Vision Pose", Pose3d.struct).publish();

    public VisionSubsystem() {
        try
        {
            fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        } catch (Exception e)
        {
            SmartDashboard.putString("bro actually errored....", e.toString());
        }

        poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, robotToBigCam);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public double getID()
    {
        return tagID;
    }

    // public EstimatedRobotPose getEstimatedPose() {
    //     if (latestResult == null) {
    //         return null;
    //     }
    //     Optional<EstimatedRobotPose> pose = poseEstimator.update(latestResult);
    //     SmartDashboard.putBoolean("EJOIJFEWOIFJW???", pose.isEmpty());
    //     SmartDashboard.putNumber("timestampritvik???", latestResult.getTimestampSeconds());

    //     posePublisher.set(pose.isPresent() ? pose.get().estimatedPose : new Pose3d(0, 2, 0, new Rotation3d(0, 0, 0)));
    //     return pose.isPresent() ? pose.get() : null;
    // }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (latestResult.hasTargets())
            return poseEstimator.update(latestResult);
        else
            return Optional.empty();
    }

    @Override
    public void periodic() 
    {
        List<PhotonPipelineResult> results = bigCamera.getAllUnreadResults();
        if (!results.isEmpty())
            latestResult = results.get(results.size() - 1);

        SmartDashboard.putBoolean("Camera is connected", bigCamera.isConnected());
        SmartDashboard.putBoolean("Camera has results", !results.isEmpty());

        Optional<EstimatedRobotPose> poses = getEstimatedGlobalPose();
        SmartDashboard.putBoolean("Is Present", poses.isPresent());
        if (poses.isPresent()) {
            Pose3d tagPose = poses.get().estimatedPose;
            Transform3d cameraToTag = latestResult.getBestTarget().getBestCameraToTarget();
            robotPose = tagPose.transformBy(cameraToTag.inverse());
        }

        posePublisher.set(robotPose);

        if (latestResult != null) {
            tagID = latestResult.hasTargets() ? latestResult.getBestTarget().getFiducialId() : -1;
            yaw = latestResult.hasTargets() ? latestResult.getBestTarget().getYaw() : 0;
            pitch = latestResult.hasTargets() ? latestResult.getBestTarget().getPitch() : 0;
            area = latestResult.hasTargets() ? latestResult.getBestTarget().getArea() : 0;
        }

    }
    
}
