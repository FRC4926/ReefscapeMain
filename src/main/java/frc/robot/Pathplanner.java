package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Pathplanner {
    public static final OurAutoBuilder auton = new OurAutoBuilder();
    public static final OurAutoBuilder fly = new OurAutoBuilder();

    public static void configure(CommandSwerveDrivetrain drivetrain) {
        try {
            auton.configure(
                () -> drivetrain.getState().Pose,   // Supplier of current robot pose
                drivetrain::resetPose,         // Consumer for seeding pose against auto
                () -> drivetrain.getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> drivetrain.setControl(
                    drivetrain.m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    AutonConstants.pathplannerTranslationPIDConstants,
                    // PID constants for rotation
                    AutonConstants.pathplannerRotationPIDConstants
                ),
                RobotConfig.fromGUISettings(),
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue,
                drivetrain // Subsystem for requirements
            );

            fly.configure(
                () -> drivetrain.getState().Pose,   // Supplier of current robot pose
                drivetrain::resetPose,         // Consumer for seeding pose against auto
                () -> drivetrain.getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> drivetrain.setControl(
                    drivetrain.m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    AutonConstants.flyTranslationPIDConstants,
                    // PID constants for rotation
                    AutonConstants.flyRotationPIDConstants
                ),
                RobotConfig.fromGUISettings(),
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue,
                drivetrain // Subsystem for requirements
            );
        } catch(Exception e) {
            System.out.println(e);
        }
    }
}
