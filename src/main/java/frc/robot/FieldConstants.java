package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


public class FieldConstants {
    //AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);


    public final static  Pose2d CORAL_STATION = new Pose2d(12.25, 5.08, Rotation2d.fromDegrees(RobotContainer.visionSubsystem.getTagPose().getRotation().getAngle()));

    
}
