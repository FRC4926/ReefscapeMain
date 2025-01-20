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

public class CameraWrapper {
    public PhotonCamera camera;
    public Transform3d robotToCam;
    public PhotonPipelineResult latestResult;
    public PhotonPoseEstimator poseEstimator;

    public CameraWrapper(String camName, Transform3d _robotToCam, AprilTagFieldLayout _fieldLayout) {
        camera = new PhotonCamera(camName);
        robotToCam = _robotToCam;
        poseEstimator = new PhotonPoseEstimator(_fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, _robotToCam.inverse());
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
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

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (!camera.isConnected() || latestResult == null || !latestResult.hasTargets())
        {
            return Optional.empty();
        } 

        var estimated = poseEstimator.update(latestResult);
        if (estimated.isEmpty()) return Optional.empty();

        SmartDashboard.putBoolean("Pose estimator is present for" + camera.getName(), estimated.isPresent());

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
}