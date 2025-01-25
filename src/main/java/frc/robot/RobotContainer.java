// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightAligner;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.FieldConstants;
import edu.wpi.first.wpilibj2.command.button.InternalButton;

public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public final SwerveRequest.RobotCentric relativeDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandXboxController driverController = new CommandXboxController(0);
    public static final Joystick operatorController = new Joystick(1);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final static VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final static LimelightAligner limelightAligner = new LimelightAligner();

    private static int reefFaceIdx = 0;
    public static int[] reefFaceIdxToOperatorButtonId = {
        7, 6, 1,
        2, 3, 8,

        9, 10
    };
    private static final StructPublisher<Pose2d> reefFacePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("reefFace pose", Pose2d.struct)
        .publish();
    public static final StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("bobget pose", Pose2d.struct)
        .publish();

    public static void setReefFaceIdx(int idx) {
        reefFaceIdx = idx;
        SmartDashboard.putNumber("reefFaceIdx", reefFaceIdx);
        reefFacePublisher.set(FieldConstants.reefFaces[reefFaceIdx]);
    }
    public static int getReefFaceIdx() {
        return reefFaceIdx;
    }
    //public Trigger logitechButtonTrigger = new Trigger(() -> logitechController.getRawButton(1));
    //public Pose2d targetPose2d;

    public RobotContainer() {
        //private final AutoChoosersd autoChooser = new AutoChooser();
        configureBindings();
    }

    private void configureBindings() { 
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //align command

        // operatorController.button(4).onTrue(new RunCommand(() -> SmartDashboard.putBoolean("PLEASE JUST WORK OKAY???", true)));
        SmartDashboard.putNumber("bob sheesh joe whatever", reefFaceIdxToOperatorButtonId.length);

        // for (int i = 0; i < reefFaceIdxToOperatorButtonId.length; i++) {
        //     final int idx = i;
        //     operatorController.button(reefFaceIdxToOperatorButtonId[idx])
        //         // .onTrue(new RunCommand(() -> setReefFaceIdx(idx)));
        //         .onTrue(new RunCommand(() -> SmartDashboard.putNumber("HELLO WORLD ! ! !", idx)));
        // }
        driverController.y().onTrue(new RunCommand(() -> drivetrain.updatedPath().schedule(), drivetrain));
        driverController.a().whileTrue(new RunCommand(() -> limelightAligner.setTagToBestTag()));
        driverController.x().whileTrue(drivetrain.applyRequest(() -> limelightAligner.align(relativeDrive)));

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.x().whileTrue(new RunCommand() -> )
        // joystick.x().whileTrue(drivetrain.generatedPath(new Pose2d(12.63, 5.60, Rotation2d.fromDegrees(-62.77))));
        
        //logitechController.button(1, null).getAsBoolean();
        //Trigger.whileTrue(logitechController.getRawButton(1));
        // joystick.x().onTrue(RobotContainer.drivetrain.updatedPath());

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("ThreeCoral");
    }
}
