// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;



import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LimelightAlignAuton extends Command {


  /** Creates a new IntakeCommand. */
  public LimelightAlignAuton() {
    addRequirements(RobotContainer.drivetrain);
    addRequirements(RobotContainer.limelightAligner);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
   // RobotContainer.drivetrain.applyRequest(() -> RobotContainer.limelightAligner.zeroDrive(RobotContainer.relativeDrive));
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("runnign!");
    RobotContainer.limelightAligner.setTagToBestTag();
    RobotContainer.drivetrain.applyRequest(() -> RobotContainer.limelightAligner.align(RobotContainer.relativeDrive));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return RobotContainer.limelightAligner.isFinishedAlign();
    //return Math.abs(strafe.getPositionError()) < 0.1;
  }
}
