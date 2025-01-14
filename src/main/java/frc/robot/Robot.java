// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  // PhotonCamera bigCamera;
  PhotonPipelineResult latestResult = null;
  double tag = 0;

  Mechanism2d mech;

  public Robot() {
    // bigCamera = new PhotonCamera("bigcam");
    m_robotContainer = new RobotContainer();


    // // the main mechanism object
    // mech = new Mechanism2d(3, 3);
    // // the mechanism root node
    // MechanismRoot2d root = mech.getRoot("climber", 2, 0);
    // var m_elevator = root.append(new MechanismLigament2d("elevator", 1.0, 90, 10, new Color8Bit(Color.kGreen)));
    // var m_wrist =
    //     m_elevator.append(
    //         new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
  }

  boolean hasResults = false;


  // double angleA = 90;
  // double angleB = 90;
  @Override
  public void robotPeriodic() {
    EstimatedRobotPose estimatedPose = m_robotContainer.visionSubsystem.getEstimatedPose();
    if (estimatedPose != null) {
      m_robotContainer.drivetrain.addVisionMeasurement(estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
    }
    // MechanismRoot2d root = mech.getRoot("climber", 2, 0);
    // var m_elevator = root.append(new MechanismLigament2d("elevator", 1.0, angleA, 10, new Color8Bit(Color.kGreen)));
    // var m_wrist =
    //     m_elevator.append(
    //         new MechanismLigament2d("wrist", 0.5, angleB, 6, new Color8Bit(Color.kPurple)));
    // SmartDashboard.putData("Mech2d", mech);

    // angleA += 0.003;
    // angleB -= 0.001;
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

    // SmartDashboard.putBoolean("Camera is connected", bigCamera.isConnected());
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
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
