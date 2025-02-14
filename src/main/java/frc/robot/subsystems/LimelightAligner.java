package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.hal.simulation.PWMDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraWrapperConstants;

public class LimelightAligner implements Subsystem {
    // private final PhotonCamera camera = new PhotonCamera("limelight");
    private final CameraWrapper camera = RobotContainer.visionSubsystem.getCameras().get(4);
    private int tagId = -1;
    private double yaw = 0.0;
    private double distanceX = 0.0;
    private double distanceY = 0.0;
    private double distanceZ = 0.0;
    private Rotation2d rotation = Rotation2d.kZero;
    private final PIDController rotationController  = makePIDFromConstants(VisionConstants.limelightRotationPIDConstants);
    private final PIDController relativeXController = makePIDFromConstants(VisionConstants.limelightRelativeXPIDConstants);
    private final PIDController relativeYController = makePIDFromConstants(VisionConstants.limelightRelativeYPIDConstants);

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


        // List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (camera.getLatestResult() != null) {
            for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
                if (target.getFiducialId() == tagId) {
                    yaw = target.getYaw();
                    target.getBestCameraToTarget();
                    Transform3d camToTag = target.getBestCameraToTarget();
                    distanceX = camToTag.getX();
                    distanceY = camToTag.getY();
                    distanceZ = camToTag.getZ();
                    rotation  = camToTag.getRotation().toRotation2d();
                    break;
                }
            }
        }

        SmartDashboard.putBoolean("Limelight is connected", camera.isConnected());
        SmartDashboard.putBoolean("latestResult is null", camera.getLatestResult() == null);
        SmartDashboard.putNumber("Limelight tag ID", tagId);
        SmartDashboard.putNumber("Limelight yaw", yaw);
        SmartDashboard.putNumber("Limelight distance X", distanceX);
        SmartDashboard.putNumber("Limelight distance Y", distanceY);
        SmartDashboard.putNumber("Limelight distance Z", distanceZ);
        SmartDashboard.putNumber("Rotation degrees", rotation.getDegrees());


    }

    public void setTagToBestTag() {
        if (camera.getLatestResult() == null) return;

        if (camera.getLatestResult().hasTargets()) {
            tagId = camera.getLatestResult().getBestTarget().getFiducialId();
        }
    }
    
    public RobotCentric zeroDrive(RobotCentric drive) {
       return drive
            .withRotationalRate(0)
            .withVelocityX(0)
            .withVelocityY(0);

    }

    public RobotCentric align(RobotCentric drive) {
        if (camera.getLatestResult() == null || !camera.getLatestResult().hasTargets())
        {
            return zeroDrive(drive);
        } else
        {
            return drive
                .withRotationalRate(-Math.signum(rotation.getDegrees())*rotationController.calculate(Math.abs(rotation.getDegrees()), 180))
                .withVelocityX(relativeXController.calculate(distanceX, 0.0))
                .withVelocityY(relativeYController.calculate(distanceY, 0.0));
        }
    }
    // TODO should LimelightAligner be added as a requirement?
    public Command alignCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive) {
        return drivetrain.applyRequest(() -> align(drive)).until(() -> isFinishedAlign());
    }

    public boolean isFinishedAlign()
    {
        //(!latestResult.hasTargets() || latestResult == null)
        return (relativeXController.atSetpoint() && relativeYController.atSetpoint() && rotationController.atSetpoint());
    }

    private PIDController makePIDFromConstants(PIDConstants constants) {
        return new PIDController(constants.kP, constants.kI, constants.kD);
    }
}
