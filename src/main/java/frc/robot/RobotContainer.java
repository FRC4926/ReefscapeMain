// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.TunerConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
// import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightAligner;
import frc.robot.subsystems.Recorder;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightAlignerDirection;
import frc.robot.Constants.ReefscapeState;
import frc.robot.reefscape.ClimberSubsystem;
import frc.robot.reefscape.Reefscape;

public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public final static double deadband = 0.1;
        /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * deadband).withRotationalDeadband(MaxAngularRate * deadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public static final SwerveRequest.RobotCentric relativeDrive = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * deadband).withRotationalDeadband(MaxAngularRate * deadband) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    Timer timer = new Timer();

    public static final CommandXboxController driverController = new CommandXboxController(0);
    public static final CommandJoystick operatorController = new CommandJoystick(1);

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static SendableChooser<Command> autonChooser;

    public final static VisionSubsystem visionSubsystem = new VisionSubsystem();
    public final static LimelightAligner limelightAligner = new LimelightAligner();
    public final static ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    public final LEDSubsystem ledSubsytem = new LEDSubsystem();

    public static final Recorder recorder = new Recorder();

    // public static final LEDSubsystem led = new LEDSubsystem();

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

        if (DriverStation.getAlliance().isPresent())
            SmartDashboard.putString("My SIDE!!!", DriverStation.getAlliance().get().toString());

        SmartDashboard.putNumber("MY RED X COORDINATE", FieldConstants.reefFacesRed[reefFaceIdx].getX());

        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            reefFacePublisher.set(FieldConstants.reefFacesRed[reefFaceIdx]);
        else
            reefFacePublisher.set(FieldConstants.reefFacesBlue[reefFaceIdx]);
    }
    public static int getReefFaceIdx() {
        return reefFaceIdx;
    }

    // public static final boolean[] allowAddVisionMeasurements = new boolean[VisionConstants.camConstants.length];
    // static {
    //     for (int i = 0; i < allowAddVisionMeasurements.length; i++) {
    //         allowAddVisionMeasurements[i] = false;
    //         SmartDashboard.putBoolean(VisionConstants.camConstants[i].name() + " is adding vision measurement",
    //             allowAddVisionMeasurements[i]);
    //     }
    // }
    // public static void toggleAllowAddVisionMeasurement(int idx) {
    //     allowAddVisionMeasurements[idx] = !allowAddVisionMeasurements[idx];
    //     SmartDashboard.putBoolean(VisionConstants.camConstants[idx].name() + " is adding vision measurement",
    //         allowAddVisionMeasurements[idx]);
    // }
    // public static void setAllowAddVisionMeasurements(boolean value) {
    //     for (int i = 0; i < allowAddVisionMeasurements.length; i++)
    //         allowAddVisionMeasurements[i] = value;
    // }

    public static boolean allowAddVisionMeasurements = true;
    public static void setAllowAddVisionMeasurements(boolean val) {
        allowAddVisionMeasurements = val;
    }

    public RobotContainer() {

        //aligning with rotation
        for (int i = 0; i < 6; i++)
        {
            NamedCommands.registerCommand("AlignLeft" + (i+1),  limelightAligner.autonRCommand(drivetrain, relativeDrive, LimelightAlignerDirection.Left, reefscape, i));
            NamedCommands.registerCommand("AlignRight" + (i+1), limelightAligner.autonRCommand(drivetrain, relativeDrive, LimelightAlignerDirection.Right, reefscape, i));
        }

        for (int i = 0; i < 6; i++)
        {
            NamedCommands.registerCommand("AlignLeft" + (i+1) + "Slow",  limelightAligner.autonRCommandSlow(drivetrain, relativeDrive, LimelightAlignerDirection.Left, reefscape, i));
            NamedCommands.registerCommand("AlignRight" + (i+1) + "Slow", limelightAligner.autonRCommandSlow(drivetrain, relativeDrive, LimelightAlignerDirection.Right, reefscape, i));
        }

        //aligning without rotation
        NamedCommands.registerCommand("AlignLeft",  limelightAligner.autonCommand(drivetrain, relativeDrive, LimelightAlignerDirection.Left, reefscape));
        NamedCommands.registerCommand("AlignRight",  limelightAligner.autonCommand(drivetrain, relativeDrive, LimelightAlignerDirection.Right, reefscape));

        NamedCommands.registerCommand("CoralPickup", reefscape.applyStateCommand(ReefscapeState.CoralStation, true, true, false)
            .alongWith(reefscape.intake.autonIntakeCommand(IntakeConstants.autonIntakeVelocity)));
            // .andThen(reefscape.applyStateCommand(ReefscapeState.Home, true, true, false)));

        NamedCommands.registerCommand("Home", limelightAligner.autonHomeSequence(drivetrain, relativeDrive, reefscape));
        NamedCommands.registerCommand("Clear", limelightAligner.autonClearSequence(drivetrain, relativeDrive, reefscape));

        NamedCommands.registerCommand("AlgaeHome", reefscape.applyStateCommand(ReefscapeState.Home, false, false, true).alongWith(reefscape.algaeZeroCommand()));
        NamedCommands.registerCommand("AlgaeL2", reefscape.applyStateCommand(ReefscapeState.AlgaeL2, false, false, true).alongWith(reefscape.algaeIntakeCommand()));
        NamedCommands.registerCommand("AlgaeL3", reefscape.applyStateCommand(ReefscapeState.AlgaeL3, false, false, true).alongWith(reefscape.algaeIntakeCommand()));
        NamedCommands.registerCommand("AlgaeBarge", reefscape.applyStateCommand(ReefscapeState.Barge, false, false, true).andThen(reefscape.algaeOuttakeCommand()));

        NamedCommands.registerCommand("SmallDrive",  limelightAligner.smallDriveCommand(drivetrain, relativeDrive, 0.2));
        NamedCommands.registerCommand("SmallDriveCoral",  limelightAligner.coralSmallDriveCommand(drivetrain, relativeDrive));

        NamedCommands.registerCommand("SmallRDrive", limelightAligner.autonSmallRDriveCommand(drivetrain, relativeDrive));
        NamedCommands.registerCommand("ZeroDrive", new InstantCommand(() -> limelightAligner.zeroDrive(relativeDrive)));

        NamedCommands.registerCommand("StartTime", new InstantCommand(() -> timer.restart()));
        NamedCommands.registerCommand("EndTime", new InstantCommand(() -> SmartDashboard.putNumber("Auton End Time", timer.get())));
        
        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Our Autonomous", autonChooser);
        // NamedCommands.registerCommand("AddVisionMeasurements", visionSubsystem.addVisionMeasurementsOnceCommand(drivetrain));
        // NamedCommands.registerCommand("PermaVision", new InstantCommand(() -> setAllowAddVisionMeasurements(true)).andThen(visionSubsystem.addVisionMeasurementsOnceCommand(drivetrain)));
        // NamedCommands.registerCommand("UnPermaVision", new InstantCommand(() -> setAllowAddVisionMeasurements(false)));
        configureBindings();
    }

    private ReefscapeState getClearState(Supplier<ReefscapeState> goalState, boolean in)
    {
        if (goalState.get().equals(ReefscapeState.Level1))
        {
            return ReefscapeState.Level1;
        } else
        {
            if(in)
                return ReefscapeState.Clear;
            else
                return ReefscapeState.Level3;
        }
    }

    private Command limelightAlignToDirection(LimelightAlignerDirection direction) {
        Command cmd = new InstantCommand(() -> limelightAligner.zeroDrive(relativeDrive))
            //.andThen(limelightAligner.autoRotateCommand(drivetrain, relativeDrive, RobotContainer::getReefFaceIdx))
            .andThen(limelightAligner.alignCommand(drivetrain, relativeDrive, direction))
            .alongWith(reefscape.applyStateCommandManual(getClearState(() -> reefscape.getLastLevel(), true), false, true, false))
            .alongWith(
                Commands.idle().until(() -> limelightAligner.distanceX <= VisionConstants.limelightElevatorDistance)
                    .andThen(reefscape.applyStateCommand(() -> reefscape.getLastLevel(), true, false, false))
            )
            .andThen(limelightAligner.smallDriveCommand(drivetrain, relativeDrive));
        return new InstantCommand(() -> limelightAligner.setTagToBestTag()).andThen(cmd);
    }

    public Command pathOnFly()
    {
        return visionSubsystem.addVisionMeasurementsOnceCommand(drivetrain)
        .alongWith(drivetrain.updatedPathCommand(() -> getReefFaceIdx()))
        .onlyWhile(() -> limelightAligner.getInterrupt());
    }

    private void configureBindings() { 
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     ).alongWith(new RunCommand(() -> limelightAligner.setInterupt(true))));

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(limelightAligner.getManualTurn() ? -driverController.getRightX() * MaxAngularRate : limelightAligner.setpointRotate(drivetrain, reefscape.isCoralInInnerIntake()))
            ).alongWith(new RunCommand(() -> limelightAligner.setInterupt(true))));

        // ternary: bool ? true : false
        visionSubsystem.setDefaultCommand(new RunCommand(() -> {
            if (allowAddVisionMeasurements)
                visionSubsystem.addVisionMeasurements(drivetrain);
        }, visionSubsystem));

        ledSubsytem.setDefaultCommand(new RunCommand(() -> {
            ledSubsytem.setLEDState(() -> reefscape.isCoralInInnerIntake(), () -> limelightAligner.canAlign(), () -> limelightAligner.isAligning());
        }, ledSubsytem));

        

        //Liam was here
        
        for (int i = 0; i < reefFaceIdxToOperatorButtonId.length; i++) {
            final int idx = i;
            operatorController.button(reefFaceIdxToOperatorButtonId[idx])
                .onTrue(new InstantCommand(() -> setReefFaceIdx(idx)).ignoringDisable(true));
        }
        // for (int i = 0; i < cameraIdxToOperatorButtonId.length; i++) {
        //     final int idx = i;
        //     operatorController.button(cameraIdxToOperatorButtonId[idx])
        //         .onTrue(new InstantCommand(() -> toggleAllowAddVisionMeasurement(idx)));
        // }


        // elevator.setDefaultCommand(elevator.moveWithVelocityCommand(() -> -operatorController.getY()));
        // operatorController.button(21).onTrue(reefscape.toggleElevatorManualCommand());
        reefscape.elevatorIsManual().whileTrue(reefscape.elevatorMoveWithVelocityCommand(() -> -operatorController.getY()));
        operatorController.button(24).onTrue(reefscape.applyStateCommand(ReefscapeState.Level2, false, false, false));
        operatorController.button(23).onTrue(reefscape.applyStateCommand(ReefscapeState.Level3, false, false, false));
        operatorController.button(22).onTrue(reefscape.applyStateCommand(ReefscapeState.Level4, false, false, false));
        operatorController.button(21).onTrue(reefscape.applyStateCommand(ReefscapeState.CoralStation, true, true, false));
        operatorController.button(20).onTrue(
            reefscape.applyStateCommand(ReefscapeState.Home, true, false, false)
                .alongWith(new WaitCommand(0.25))
                .andThen(reefscape.applyStateCommand(ReefscapeState.Home, false, true, false))
        );

        //dealgaeify
        operatorController.button(16).onTrue(reefscape.applyStateCommand(ReefscapeState.DeAlgaeL2, true, true, false));
        operatorController.button(15).onTrue(reefscape.applyStateCommand(ReefscapeState.DeAlgaeL3, true, true, false));

        //algae mech states
        // operatorController.button(17).onTrue(reefscape.applyStateCommand(ReefscapeState.AlgaeL2, true, false, true));
        // operatorController.button(18).onTrue(reefscape.applyStateCommand(ReefscapeState.AlgaeL3, true, false, true));
        // operatorController.button(19).onTrue(reefscape.applyStateCommand(ReefscapeState.Barge, true, false, true));
        
        //algae intake states
        // operatorController.button(9).onTrue(reefscape.algaeIntakeCommand());
        // operatorController.button(10).onTrue(reefscape.algaeOuttakeCommand());
        // operatorController.button(9).negate().and(operatorController.button(10).negate()).onTrue(reefscape.algaeZeroCommand());

        //L1
        operatorController.button(5).onTrue(reefscape.outtakeCommand());
        operatorController.button(4).onTrue(reefscape.applyStateCommand(ReefscapeState.Level1, false, false, false));


        
        // When left trigger pressed, pivot moved to last known level
        driverController.leftTrigger(0.2).onTrue(
            reefscape.applyStateCommand(() -> reefscape.getLastLevel(), false, true, false)
            .andThen(reefscape.levelCommand()));
        // driverController.leftTrigger(0.2).onFalse(reefscape.applyStateCommandManual(ReefscapeState.Level3, false, true, false));
        driverController.leftTrigger(0.2).onFalse(reefscape.applyStateCommandManual(getClearState(() -> reefscape.getLastLevel(), false), false, true, false));

        driverController.rightTrigger(0.2).onTrue(reefscape.intakeCommand());
        
        //zero command
        driverController.rightTrigger(0.2).negate().and(driverController.leftTrigger(0.2).negate())
            .whileTrue(reefscape.zeroCommand());


        //climbing
        operatorController.button(11).onTrue(climberSubsystem.climbForward());
        operatorController.button(12).onTrue(climberSubsystem.climbBack());

        operatorController.button(11).negate().and(operatorController.button(12).negate())
         .whileTrue(climberSubsystem.climbZero());
        operatorController.button(13).onTrue(climberSubsystem.climbZero());

        // driverController.y().onTrue(pathOnFly());
        driverController.y().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.a().onTrue(new InstantCommand(() -> limelightAligner.setInterupt(false)));
        driverController.x().onTrue(limelightAlignToDirection(LimelightAlignerDirection.Left).onlyWhile(() -> limelightAligner.getInterrupt()));
        driverController.b().onTrue(limelightAlignToDirection(LimelightAlignerDirection.Right).onlyWhile(() -> limelightAligner.getInterrupt()));
        // driverController.rightBumper().onTrue(limelightAligner.autoRotateTeleopCommand(drivetrain, drive, RobotContainer::getReefFaceIdx, driverController::getLeftX, driverController::getLeftY, MaxSpeed).onlyWhile(() -> limelightAligner.getInterrupt()));
        
        driverController.rightBumper().whileTrue(drivetrain.applyRequest(() -> limelightAligner.driveRobotRelative(relativeDrive, 
            -driverController.getLeftY()*MaxSpeed, 
            -driverController.getLeftX()*MaxSpeed, 
            -driverController.getRightX()*MaxAngularRate)));
        
        // driverController.leftBumper().whileTrue(drivetrain.applyRequest(() ->
        //     drive.withVelocityX(driverController.getLeftY() * MaxSpeed * 0.1) // Drive forward with negative Y (forward)
        //         .withVelocityY(driverController.getLeftX() * MaxSpeed * 0.1) // Drive left with negative X (left)
        //         .withRotationalRate(-driverController.getRightX() * MaxAngularRate * 0.1) // Drive counterclockwise with negative X (left)
        // ));

        //-0.1*(x-1)^2+0.1
        driverController.leftBumper().whileTrue(drivetrain.applyRequest(() ->
            drive.withVelocityX(driverController.getLeftY() * MaxSpeed * 0.1) // Drive forward with negative Y (forward)
            .withVelocityY(driverController.getLeftX() * MaxSpeed * 0.1) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate * 0.3) // Drive counterclockwise with negative X (left)
            ));

        
        
        driverController.start().onTrue(new InstantCommand(() -> limelightAligner.toggleTurn()));


        // driverController.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.x().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        //return new PathPlannerAuto("ThreeCoralAuto");
        return (new PathPlannerAuto(autonChooser.getSelected()).withTimeout(15)).andThen(new InstantCommand(() ->limelightAligner.zeroDrive(relativeDrive)));
        // PathConstraints constraints = new PathConstraints(
        //     4,5,
        //     Units.degreesToRadians(540), Units.degreesToRadians(720));
        // return Pathplanner.pathfindToPose(FieldConstants.reefFacesRed[0], constraints, 0.0);
    }
}
