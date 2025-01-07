// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutonAlignCommand extends Command {

  PIDController strafe = new PIDController(0.2, 0, 0);
  PIDController turn = new PIDController(0.1, 0, 0);
  PhotonCamera bigCamera = new PhotonCamera("bigcam");

  /** Creates a new IntakeCommand. */
  public AutonAlignCommand() {
    addRequirements(RobotContainer.drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    strafe.setSetpoint(0);
    strafe.setTolerance(0);

    turn.setSetpoint(0);
    turn.setTolerance(0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double speed = -strafe.calculate(bigCamera.getLatestResult().hasTargets() ? bigCamera.getLatestResult().getBestTarget().getYaw(): 0);
      
      RobotContainer.drivetrain.applyRequest(() -> RobotContainer.drive.withVelocityX(0) // Drive forward with negative Y (forward)
        .withVelocityY(speed) // Drive left with negative X (left)
        .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      );
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(strafe.getPositionError()) < 0.1;
  }
}
