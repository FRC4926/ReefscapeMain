// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.CameraWrapper;
import frc.robot.subsystems.VisionSubsystem;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    // PhotonCamera bigCamera;
    PhotonPipelineResult latestResult = null;
    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("bob", Pose2d.struct)
            .publish();
    // private final StructPublisher<Pose2d> photonPublisher =
    // NetworkTableInstance.getDefault()
    // .getStructTopic("joe", Pose2d.struct)
    // .publish();

    public Robot() {
        // bigCamera = new PhotonCamera("bigcam");
        Pathfinding.setPathfinder(new RemoteADStar());
        m_robotContainer = new RobotContainer();
    }

    boolean hasResults = false;

    @Override
    public void robotPeriodic() {
        // for (int i = 0; i < RobotContainer.reefFaceIdxToOperatorButtonId.length; i++) {
        //     if (RobotContainer.operatorController.getRawButtonPressed(RobotContainer.reefFaceIdxToOperatorButtonId[i])) {
        //         m_robotContainer.setReefFaceIdx(i);
        //         break;
        //     }
        // }
        // for (int i = 0; i < m_robotContainer.logitechController.getButtonCount();
        // i++) {
        // SmartDashboard.putBoolean("Button #" + (i + 1),
        // m_robotContainer.logitechController.getRawButton(i + 1));
        // }

        // var result = bigCamera.getLatestResult();
        // var results = bigCamera.getLatestResult();
        // if (!results.hasTargets()) hasResults = true;

        // SmartDashboard.putNumber("Number of unread results", results.size());
        // SmartDashboard.putBoolean("Were there results at one point", hasResults);

        // SmartDashboard.putNumber("Camera pipeline index",
        // bigCamera.getPipelineIndex());
        // SmartDashboard.putBoolean("Camera has targets", result.hasTargets());
        // if (result.hasTargets()) {
        // var target = result.getBestTarget();
        // SmartDashboard.putNumber("Target yaw", target.getYaw());
        // SmartDashboard.putNumber("Target pitch", target.getPitch());
        // SmartDashboard.putNumber("April tag Small Index", target.getFiducialId());
        // } else
        // {
        // SmartDashboard.putNumber("Target yaw", -1);
        // SmartDashboard.putNumber("Target pitch", -1);
        // SmartDashboard.putNumber("April tag Small Index", -1);
        // }

        // RobotContainer.visionSubsystem.setReferencePose();

        // List<CameraWrapper> camWrappers = RobotContainer.visionSubsystem.getCameras();

        // for (int i = 0; i < camWrappers.size(); i++) {
        //     CameraWrapper cam = camWrappers.get(i);
        //     if (cam.isConnected()) {
        //         Optional<EstimatedRobotPose> estimatedPose = camWrappers.get(i).getEstimatedGlobalPose();
        //         if (estimatedPose.isPresent()) {
        //             EstimatedRobotPose poseCam = estimatedPose.get();
        //             // photonPublisher.set(poseCam.estimatedPose.toPose2d());
        //             RobotContainer.drivetrain.addVisionMeasurement(poseCam.estimatedPose.toPose2d(),
        //                     Utils.fpgaToCurrentTime(poseCam.timestampSeconds),
        //                     new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] { 1.2, 1.2, 0.99 }));
        //         }
        //     }
        // }

        // Optional<EstimatedRobotPose> estimatedPoseFront =
        // RobotContainer.visionSubsystem.getEstimatedGlobalPoseFront();
        // Optional<EstimatedRobotPose> estimatedPoseBack =
        // RobotContainer.visionSubsystem.getEstimatedGlobalPoseBack();

        // SmartDashboard.putBoolean("estimatedPose is present front",
        // estimatedPoseFront.isPresent());
        // SmartDashboard.putBoolean("estimatedPose is present back",
        // estimatedPoseBack.isPresent());

        // if (estimatedPoseFront.isPresent()) {
        // EstimatedRobotPose poseFront = estimatedPoseFront.get();
        // photonPublisher.set(poseFront.estimatedPose.toPose2d());
        // RobotContainer.drivetrain.addVisionMeasurement(poseFront.estimatedPose.toPose2d(),
        // Utils.fpgaToCurrentTime(poseFront.timestampSeconds), new Matrix<N3, N1>
        // (Nat.N3(), Nat.N1(), new double[] {0.8, 0.8, 0.99}));
        // }

        // if (estimatedPoseBack.isPresent()) {
        // EstimatedRobotPose poseBack = estimatedPoseBack.get();
        // photonPublisher.set(poseBack.estimatedPose.toPose2d());
        // RobotContainer.drivetrain.addVisionMeasurement(poseBack.estimatedPose.toPose2d(),
        // Utils.fpgaToCurrentTime(poseBack.timestampSeconds), new Matrix<N3, N1>
        // (Nat.N3(), Nat.N1(), new double[] {0.8, 0.8, 0.99}));
        // }

        posePublisher.set(RobotContainer.drivetrain.getState().Pose);



        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {

        // RobotContainer.drivetrain.targetChange();
        // List<PhotonPipelineResult> results = bigCamera.getAllUnreadResults();
        // if (!results.isEmpty())
        // latestResult = results.get(results.size() - 1);

        // SmartDashboard.putNumber("April Tag ID",
        // m_robotContainer.visionSubsystem.getIDFront());
        // SmartDashboard.putBoolean("Camera has results", !results.isEmpty());
        // boolean hasTargets = false;

        // if (latestResult != null) {
        // SmartDashboard.putNumber("Results", latestResult.hasTargets() ?
        // latestResult.getBestTarget().getFiducialId() : -1);
        // }

        // if (!results.isEmpty()) {
        // for (var result : results) {
        // if (result.hasTargets())
        // hasTargets = true;
        // }
        // }

        // try
        // {
        // SmartDashboard.putNumber("Results", results.get((results.size() -
        // 1)).getBestTarget().getFiducialId());
        // tag = results.get((results.size() - 1)).getBestTarget().getFiducialId();
        // } catch (Exception e)
        // {
        // if (hasTargets)
        // {
        // SmartDashboard.putNumber("Results", tag);

        // } else
        // {
        // SmartDashboard.putNumber("Results", -1);
        // }
        // }
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        // PathConstraints conts = new PathConstraints(
        // 3.0, 4.0,
        // Units.degreesToRadians(540), Units.degreesToRadians(720));

        // PathPlannerPath shees = new
        // PathPlannerPath(PathPlannerPath.waypointsFromPoses(RobotContainer.drivetrain.getState().Pose,
        // new Pose2d(16.02, 0.76, Rotation2d.fromDegrees(127))), conts, null, new
        // GoalEndState(0.0, Rotation2d.fromDegrees(127)));
        // AutoBuilder.followPath(shees).schedule();

        Pose2d targetPose = new Pose2d(12.63, 5.60, Rotation2d.fromDegrees(-62.77));
        // Pose2d targetPose = new Pose2d(12.88, 6.17, new
        // Rotation2d(-109.81*Math.PI/180));

        // Pose2d targetPose = RobotContainer.visionSubsystem.getTagPose().toPose2d();

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                1, 1,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0);

        CommandScheduler.getInstance().schedule(pathfindingCommand);
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
