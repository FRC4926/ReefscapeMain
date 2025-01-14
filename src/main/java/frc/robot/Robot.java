// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.VisionSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  PhotonCamera bigCamera;
  PhotonPipelineResult latestResult = null;

  public Robot() {
    bigCamera = new PhotonCamera("bigcam");
    Pathfinding.setPathfinder(new LocalADStar());
    m_robotContainer = new RobotContainer();
  }

  boolean hasResults = false;
  @Override
  public void robotPeriodic() {
    // var result = bigCamera.getLatestResult();
    // var results = bigCamera.getLatestResult();
    // if (!results.hasTargets()) hasResults = true;


    //SmartDashboard.putNumber("Number of unread results", results.size());
    // SmartDashboard.putBoolean("Were there results at one point", hasResults);

    // SmartDashboard.putNumber("Camera pipeline index", bigCamera.getPipelineIndex());
    // SmartDashboard.putBoolean("Camera has targets", result.hasTargets());
    // if (result.hasTargets()) {
    //   var target = result.getBestTarget();
    //   SmartDashboard.putNumber("Target yaw", target.getYaw());
    //   SmartDashboard.putNumber("Target pitch", target.getPitch());
    //   SmartDashboard.putNumber("April tag Small Index", target.getFiducialId());
    // } else
    // {
    //   SmartDashboard.putNumber("Target yaw", -1);
    //   SmartDashboard.putNumber("Target pitch", -1);
    //   SmartDashboard.putNumber("April tag Small Index", -1);
    // }


    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }


  @Override
  public void teleopPeriodic() {
    // List<PhotonPipelineResult> results = bigCamera.getAllUnreadResults();
    // if (!results.isEmpty())
    //   latestResult = results.get(results.size() - 1);

    SmartDashboard.putNumber("April Tag ID", m_robotContainer.visionSubsystem.getID());
    // SmartDashboard.putBoolean("Camera has results", !results.isEmpty());
    // boolean hasTargets = false;

    // if (latestResult != null) {
    //   SmartDashboard.putNumber("Results", latestResult.hasTargets() ? latestResult.getBestTarget().getFiducialId() : -1);
    // }

    // if (!results.isEmpty()) {
    //   for (var result : results) {
    //     if (result.hasTargets())
    //       hasTargets = true;
    //   }
    // }

    // try
    // {
    //   SmartDashboard.putNumber("Results", results.get((results.size() - 1)).getBestTarget().getFiducialId());
    //   tag = results.get((results.size() - 1)).getBestTarget().getFiducialId();
    // } catch (Exception e)
    // {
    //   if (hasTargets)
    //   {
    //     SmartDashboard.putNumber("Results", tag);

    //   } else
    //   {
    //     SmartDashboard.putNumber("Results", -1);
    //   }
    // }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
        //     PathConstraints conts = new PathConstraints(
        //     3.0, 4.0,
        //     Units.degreesToRadians(540), Units.degreesToRadians(720));

        // PathPlannerPath shees = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(RobotContainer.drivetrain.getState().Pose, new Pose2d(16.02, 0.76, Rotation2d.fromDegrees(127))), conts, null,  new GoalEndState(0.0, Rotation2d.fromDegrees(127)));
        // AutoBuilder.followPath(shees).schedule();


        //Pose2d targetPose = new Pose2d(12.25, 5.08, Rotation2d.fromDegrees(RobotContainer.visionSubsystem.getTagPose().getRotation().getAngle()*(180/Math.PI)));
        Pose2d targetPose = RobotContainer.visionSubsystem.getTagPose().toPose2d();


        

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
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
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
