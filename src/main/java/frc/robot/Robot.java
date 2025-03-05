// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ReefscapeState;
import frc.robot.subsystems.ThroughboreEncoder;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    // PhotonCamera bigCamera;
    PhotonPipelineResult latestResult = null;
    // private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
    //         .getStructTopic("bob", Pose2d.struct)
    //         .publish();
    // private final StructPublisher<Pose2d> photonPublisher =
    // NetworkTableInstance.getDefault()
    // .getStructTopic("joe", Pose2d.struct)
    // .publish();

    Timer timer = new Timer();
    Timer timer2 = new Timer();

    // static Alliance alliance = null;

    // AnalogInput myInput = new AnalogInput(1);
    // DutyCycle dc = new DutyCycle(new DigitalInput(9));
    // DigitalInput myInput = new DigitalInput(9);

    ThroughboreEncoder encoder = new ThroughboreEncoder(new DigitalInput(0), new DigitalInput(1), new DigitalInput(2), 1.0);
    // Encoder enc2 = new Encoder(new DigitalInput(0), new DigitalInput(1));

    public Robot() {

        timer.start();
        timer2.start();
        // bigCamera = new PhotonCamera("bigcam");
        Pathfinding.setPathfinder(new RemoteADStar());
        //PathfindingCommand.warmupCommand().schedule();
        m_robotContainer = new RobotContainer();

        // makeMech();

        // PWMJNI.getPWMPosition(0)
        // myInput.setAverageBits(16);
    }

    boolean hasResults = false;

    // public void makeMech() {
    //     MechanismRoot2d myRoot = myMech.getRoot("myRoot", 5.0, 0.0);
    //     MechanismLigament2d myElevator = myRoot.append(new MechanismLigament2d("myElevator", 5.0, 90.0, 10.0, new Color8Bit(Color.kGreen)));
    // }

 
    @Override
    public void robotPeriodic() {
        // SmartDashboard.putNumber("Relative Position", encoder.getRelativePosition());
        // SmartDashboard.putNumber("Absolute Position", encoder.getAbsolutePosition());

        // SmartDashboard.putData("myMech", myMech);


        // double a = RobotContainer.drivetrain.getCurrent(), b = RobotContainer.reefscape.elevator.getCurrent(),
        //     c = RobotContainer.reefscape.pivot.getCurrent(), d = RobotContainer.reefscape.intake.getCurrent();
        //     // e = RobotContainer.climberSystem.getCurrent();

        // SmartDashboard.putNumber("CURRENT: Drivetrain", a);
        // SmartDashboard.putNumber("CURRENT: Elevator", b);
        // SmartDashboard.putNumber("CURRENT: Pivot", c);
        // SmartDashboard.putNumber("CURRENT: Intake", d);
        // // SmartDashboard.putNumber("CURRENT: Climber", e);
        // SmartDashboard.putNumber("CURRENT: Total", a + b + c + d);


        SmartDashboard.putNumber("Elevator Position", RobotContainer.reefscape.elevator.getPosition());
        SmartDashboard.putNumber("Pivot Position", RobotContainer.reefscape.pivot.getConvertedPosition());
        // SmartDashboard.putNumber("Pivot Converted Position", RobotContainer.reefscape.pivot.getPosition()/32.0*360.0)
        // ;
         SmartDashboard.putNumber("Climber 1 Position", RobotContainer.climberSystem.getClimb1Position());
        // SmartDashboard.putNumber("Climber 2 Position", RobotContainer.climberSystem.getClimb2Position());
        // SmartDashboard.putNumber("Climber 1 Position Converted", RobotContainer.climberSystem.getClimb2Converted());

        // SmartDashboard.putNumber("Climber 1 Current", RobotContainer.climberSystem.getCurrent1());
        SmartDashboard.putNumber("IntakeSensor Value", RobotContainer.reefscape.intake.innerProximitySensor.getAverageVoltage());
        // SmartDashboard.putBoolean(" Vision Measurement added", RobotContainer.allowAddVisionMeasurements);
        // SmartDashboard.putBoolean("Is Coral in?", RobotContainer.reefscape.intake.isCoralInInnerIntake());

        // SmartDashboard.putBoolean("is finished lime", RobotContainer.limelightAligner.isFinishedAlign());

    
        //SmartDashboard.putNumber("Refined Position", encoder.getRefinedPosition());
        // SmartDashboard.putNumber("Relative Position 2", enc2.getDistance());
        // SmartDashboard.putNumber("Color Sensor My Input", dc.getOutput() * 256);
        // SmartDashboard.putNumber("Color Sensor My Input", PWMJNI.getPWMPosition(0));
        // SmartDashboard.putNumber("Color Sensor My Input 2", PWMJNI.getPWMSpeed(0));

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


        // SmartDashboard.putNumber("loop time", timer.get());
        // timer.reset();

        // timer2.reset();

        // posePublisher.set(RobotContainer.drivetrain.getState().Pose);
        CommandScheduler.getInstance().run();

        // SmartDashboard.putNumber("period time", timer2.get());

        SmartDashboard.putBoolean("teleop", DriverStation.isTeleop());
        SmartDashboard.putBoolean("teleop enabled", DriverStation.isTeleopEnabled());
        SmartDashboard.putString("state", RobotContainer.reefscape.getLastLevel().toString());
        SmartDashboard.putString("state fr", RobotContainer.reefscape.getState().toString());
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
        RobotContainer.setAllowAddVisionMeasurements(true);
        // RobotContainer.visionSubsystem.addVisionMeasurements(RobotContainer.drivetrain);
        // RobotContainer.setAllowAddVisionMeasurements(false);

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
        RobotContainer.setAllowAddVisionMeasurements(true);

        RobotContainer.reefscape.applyState(ReefscapeState.Home);
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

    }

    @Override
    public void teleopPeriodic() {

        SmartDashboard.putNumber("Left PID output", RobotContainer.reefscape.elevator.leftMotor.getClosedLoopOutput().getValueAsDouble());
        SmartDashboard.putNumber("Right PID output",RobotContainer.reefscape.elevator.rightMotor.getClosedLoopOutput().getValueAsDouble());

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
