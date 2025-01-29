package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class LimelightAligner extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("limelight");
    private PhotonPipelineResult latestResult = null;
    private int tagId = -1;
    private double yaw = 0.0;
    private double distanceX = 0.0;
    private double distanceY = 0.0;
    private double distanceZ = 0.0;
    private final PIDController rotationController  = makePIDFromConstants(VisionConstants.limelightRotationPIDConstants);
    private final PIDController relativeXController = makePIDFromConstants(VisionConstants.limelightRelativeXPIDConstants);
    private final PIDController relativeYController = makePIDFromConstants(VisionConstants.limelightRelativeYPIDConstants);
    private final PIDController relativeZController = new PIDController(1.0, 0.0, 0.0);

    public LimelightAligner() {

    }

    public int getTagId() {
        return tagId;
    }

    // private double makeDistance(double pitch) {
    //     return PhotonUtils.calculateDistanceToTargetMeters(
    //         Units.inchesToMeters(13),
    //         Units.inchesToMeters(12.125),
    //         Units.degreesToRadians(-13),
    //         Units.degreesToRadians(pitch));
    // }

    @Override
    public void periodic() {
        if (!camera.isConnected()) return;

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            latestResult = results.get(results.size() - 1);

            for (PhotonTrackedTarget target : latestResult.getTargets()) {
                if (target.getFiducialId() == tagId) {
                    yaw = target.getYaw();
                    target.getBestCameraToTarget();
                    Transform3d camToTag = target.getBestCameraToTarget();
                    distanceX = camToTag.getX();
                    distanceY = camToTag.getY();
                    distanceZ = camToTag.getRotation().toRotation2d().toDegrees();
                    break;
                }
            }
        }

        SmartDashboard.putBoolean("Limelight is connected", camera.isConnected());
        SmartDashboard.putBoolean("latestResult is null", latestResult == null);
        SmartDashboard.putNumber("Limelight tag ID", tagId);
        SmartDashboard.putNumber("Limelight yaw", yaw);
        SmartDashboard.putNumber("Limelight distance X", distanceX);
        SmartDashboard.putNumber("Limelight distance Y", distanceY);
        SmartDashboard.putNumber("Limelight distance Z", distanceZ);


    }

    public void setTagToBestTag() {
        if (latestResult == null) return;

        if (latestResult.hasTargets()) {
            tagId = latestResult.getBestTarget().getFiducialId();
        }
    }

    public RobotCentric align(RobotCentric drive) {
        return drive
            .withRotationalRate(rotationController.calculate(yaw, 0.0))
            .withVelocityX(relativeXController.calculate(distanceX, 0.05))
            .withVelocityY(relativeYController.calculate(distanceY, 0.0));
    }

    private PIDController makePIDFromConstants(PIDConstants constants) {
        return new PIDController(constants.kP, constants.kI, constants.kD);
    }
}
