package frc.robot.subsystems;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {

    PhotonCamera bigCamera;
    PhotonPipelineResult latestResult = null;

    double tagID = 0;
    double yaw = 0;
    double pitch = 0;
    double area = 0;

    public VisionSubsystem() {
        bigCamera = new PhotonCamera("bigcam");
    }

    public double getID()
    {
        return tagID;
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
