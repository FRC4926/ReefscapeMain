// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightAligner;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightAlignerDirection;
import frc.robot.Constants.ReefscapeState;
import frc.robot.reefscape.Reefscape;

public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
            
        /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public static final SwerveRequest.RobotCentric relativeDrive = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandXboxController driverController = new CommandXboxController(0);
    public static final CommandJoystick operatorController = new CommandJoystick(1);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final static VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final static LimelightAligner limelightAligner = new LimelightAligner();

    public static final Reefscape reefscape = new Reefscape();

    private static int reefFaceIdx = 0;
    public static final int[] reefFaceIdxToOperatorButtonId = {
        7, 6, 1,
        2, 3, 8,

        9, 10
    };
    public static final int[] cameraIdxToOperatorButtonId = {
        17, 18, // up and down
        19, 20  // left and right
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

    public static final boolean[] allowAddVisionMeasurements = new boolean[VisionConstants.camConstants.length];
    static {
        for (int i = 0; i < allowAddVisionMeasurements.length; i++) {
            allowAddVisionMeasurements[i] = true;
            SmartDashboard.putBoolean(VisionConstants.camConstants[i].name() + " is adding vision measurement",
                allowAddVisionMeasurements[i]);
        }
    }
    public static void toggleAllowAddVisionMeasurement(int idx) {
        allowAddVisionMeasurements[idx] = !allowAddVisionMeasurements[idx];
        SmartDashboard.putBoolean(VisionConstants.camConstants[idx].name() + " is adding vision measurement",
            allowAddVisionMeasurements[idx]);
    }
    //public Trigger logitechButtonTrigger = new Trigger(() -> logitechController.getRawButton(1));
    //public Pose2d targetPose2d;

    public RobotContainer() {
        //private final AutoChoosersd autoChooser = new AutoChooser();
        NamedCommands.registerCommand("AlignLeft",  limelightAligner.autonCommand(drivetrain, relativeDrive, LimelightAlignerDirection.Left));
        NamedCommands.registerCommand("AlignRight", limelightAligner.autonCommand(drivetrain, relativeDrive, LimelightAlignerDirection.Right));
        configureBindings();
    }

    private boolean posesClose(Pose2d a, Pose2d b, double threshold) {
        Translation2d between = a.getTranslation().minus(b.getTranslation());
        double distSquared = Math.pow(between.getX(), 2) + Math.pow(between.getY(), 2);

        return distSquared <= threshold*threshold;
    }
    // returns -1 if not near coral station, and reef face index otherwise
    private int nearCoralStation() {
        Pose2d pose = drivetrain.getState().Pose;
        if (posesClose(pose, FieldConstants.reefFaces[6], FieldConstants.coralStationToRobotThreshold))
            return 6;
        if (posesClose(pose, FieldConstants.reefFaces[7], FieldConstants.coralStationToRobotThreshold))
            return 7;
        return -1;
    }
    private boolean shouldSetStateToCoralStation() {
        int idx = nearCoralStation();
        return (idx != -1) && (reefFaceIdx == idx);
    }

    private Command limelightAlignToDirection(LimelightAlignerDirection direction) {
        return limelightAligner.alignCommand(drivetrain, relativeDrive, direction)
            .alongWith(reefscape.applyStateCommand(() -> reefscape.getLastLevel(), true, true, false))
            .andThen(reefscape.applyStateCommand(() -> reefscape.getLastLevel(), true, true, true));
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
            ).alongWith(new RunCommand(() -> drivetrain.setInterupt(true))));

            visionSubsystem.setDefaultCommand(new RunCommand(() -> {
            EstimatedRobotPose[] poses = visionSubsystem.getEstimatedGlobalPoses();
            for (int i = 0; i < poses.length; i++) {
                if ((poses[i] != null) && allowAddVisionMeasurements[i])
                    drivetrain.addVisionMeasurement(
                        poses[i].estimatedPose.toPose2d(),
                        Utils.fpgaToCurrentTime(poses[i].timestampSeconds),
                        new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {
                            VisionConstants.kalmanPositionStdDev, VisionConstants.kalmanPositionStdDev,
                            VisionConstants.kalmanRotationStdDev
                        })
                    );
            }
        }, visionSubsystem));

        //align command

        // operatorController.button(4).onTrue(new RunCommand(() -> SmartDashboard.putBoolean("PLEASE JUST WORK OKAY???", true)));
        SmartDashboard.putNumber("bob sheesh joe whatever", reefFaceIdxToOperatorButtonId.length);

        for (int i = 0; i < reefFaceIdxToOperatorButtonId.length; i++) {
            final int idx = i;
            operatorController.button(reefFaceIdxToOperatorButtonId[idx])
                .onTrue(new InstantCommand(() -> setReefFaceIdx(idx)).ignoringDisable(true));
        }
        for (int i = 0; i < cameraIdxToOperatorButtonId.length; i++) {
            final int idx = i;
            operatorController.button(cameraIdxToOperatorButtonId[idx])
                .onTrue(new InstantCommand(() -> toggleAllowAddVisionMeasurement(idx)));
        }


        // elevator.setDefaultCommand(elevator.moveWithVelocityCommand(() -> -operatorController.getY()));
        reefscape.elevatorIsManual().whileTrue(reefscape.elevatorMoveWithVelocityCommand(() -> -operatorController.getY()));
        operatorController.button(24).onTrue(reefscape.applyStateCommand(ReefscapeState.Level2, false, false, false));
        operatorController.button(23).onTrue(reefscape.applyStateCommand(ReefscapeState.Level3, false, false, false));
        operatorController.button(22).onTrue(reefscape.applyStateCommand(ReefscapeState.Level4, false, false, false));
        operatorController.button(21).onTrue(reefscape.toggleElevatorManualCommand());

        new Trigger(() -> shouldSetStateToCoralStation()).onTrue(reefscape.applyStateCommand(ReefscapeState.CoralStation));
        new Trigger(() -> (reefscape.getState() == ReefscapeState.CoralStation)) //&& (reefscape.coralInInnerIntake()))
            .onTrue(reefscape.applyStateCommand(ReefscapeState.Home));
        // reefscapeSubsystem.setDefaultCommand(coralStationCommand);
        // TODO I changed this to `InstantCommand` because this only runs it once, while `RunCommand` runs it every period.
        // Does this make it stutter less?
        driverController.y().onTrue(new InstantCommand(() -> drivetrain.updatedPath().schedule(), drivetrain));
        driverController.a().whileTrue(new RunCommand(() -> limelightAligner.setTagToBestTag()));
        driverController.x().onTrue(limelightAlignToDirection(LimelightAlignerDirection.Left));
        driverController.b().onTrue(limelightAlignToDirection(LimelightAlignerDirection.Right));
        new Trigger(() -> reefscape.getState().isLevel()) //&& (!reefscape.coralInOuterIntake()))
            .onTrue(reefscape.applyStateCommand(ReefscapeState.Home));
        // driverController.b().whileTrue(new RunCommand(()-> drivetrain.setInterupt(false)));
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
        return new PathPlannerAuto("AlignAuton");
    }
}
