package frc.robot.subsystems;

import java.net.CacheRequest;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.reefscape.Reefscape;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightAlignerDirection;
import frc.robot.Constants.ReefscapeState;
import frc.robot.Constants.VisionConstants;

public class LimelightAligner extends SubsystemBase {
    // private final PhotonCamera camera = new PhotonCamera("limelight");
    private CameraWrapper camera = null;

    private int tagId = -1;
    private double yaw = 0.0;
    public double distanceX = 0.0;
    public double distanceY = 0.0;
    private double distanceZ = 0.0;
    private Rotation2d rotation = Rotation2d.kZero;
    private final PIDController rotationController  = makePIDFromConstants(VisionConstants.limelightRotationPIDConstants);
    private final PIDController rotationController2  = makePIDFromConstants(VisionConstants.limelightRotationPIDConstants);
    private final PIDController rotationController3  = makePIDFromConstants(VisionConstants.limelightRotationPIDConstants);
    private final PIDController relativeXController = makePIDFromConstants(VisionConstants.limelightRelativeXPIDConstants);
    private final PIDController relativeYController = makePIDFromConstants(VisionConstants.limelightRelativeYPIDConstants);

    Alliance side = null;
    // Timer timeSmall = new Timer();

    private boolean interupt = true;
    private boolean manualTurn = false;

    private Pose2d targetPose = new Pose2d();

    public LimelightAligner() {
        for (var cam : RobotContainer.visionSubsystem.getCameras()) {
            if (cam.getCamera().getName().equals("limelight")) {
                camera = cam;
                break;
            }
        }

        side = DriverStation.getAlliance().orElse(Alliance.Red);
        rotationController.setTolerance(0.1);
        rotationController2.setTolerance(5);
        rotationController3.setTolerance(3);
        relativeXController.setTolerance(0.01);
        relativeYController.setTolerance(0.01);

        rotationController2.enableContinuousInput(-180, 180);
        rotationController3.enableContinuousInput(-180, 180);

    }

    public boolean getInterrupt()
    {
        return interupt;
    }

    public void setInterupt(boolean val)
    {
        interupt = val;
    }

    public int getTagId() {
        return tagId;
    }

    // private double makeDistance(double pitch) {
    //     return PhotonUtils.calculateDistanceToTargetMeters(
    //         Units.inchesToMeters(13),
    //         Units.inchesToMeters(12.125),
    //         Units.degreesToRadians(-13),
    //         Units.degreesToRadians(pitch));
    // }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Is fin", isFinishedRot());
        if ((camera == null) || !camera.isConnected()) {
            SmartDashboard.putBoolean("COULD NOT CONNECT", true);
            return;
        }


        // List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (camera.getLatestResult() != null) {
            for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
                if (target.getFiducialId() == tagId) {
                    yaw = target.getYaw();
                    target.getBestCameraToTarget();
                    Transform3d camToTag = target.getBestCameraToTarget();
                    // camToTagPose = camToTag;
                    distanceX = camToTag.getX();
                    distanceY = camToTag.getY();
                    distanceZ = camToTag.getZ();
                    rotation  = camToTag.getRotation().toRotation2d();
                    break;
                }
            }
        }

        SmartDashboard.putBoolean("Limelight is connected", camera.isConnected());
        SmartDashboard.putBoolean("latestResult is null", camera.getLatestResult() == null);
        SmartDashboard.putNumber("Limelight tag ID", tagId);
        SmartDashboard.putNumber("Limelight yaw", yaw);
        SmartDashboard.putNumber("Limelight distance X", distanceX);
        SmartDashboard.putNumber("Limelight distance Y", distanceY);
        SmartDashboard.putNumber("Limelight distance Z", distanceZ);
        SmartDashboard.putNumber("Rotation degrees", rotation.getDegrees());
    }

    public Command smallDriveCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive) {
        return smallDriveCommand(drivetrain, drive, AutonConstants.smallDriveTimeoutSeconds);
    }

    public Command smallDriveCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, double timeoutSeconds)
    {
        // timeSmall.restart();
        return drivetrain.applyRequest(() -> drive
            .withRotationalRate(0)
            .withVelocityX(AutonConstants.smallDriveVelocity)
            .withVelocityY(0)).withTimeout(timeoutSeconds);
    }

    public Command autonSmallDriveCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive)
    {
        // timeSmall.restart();
        return drivetrain.applyRequest(() -> drive
            .withRotationalRate(0)
            .withVelocityX(AutonConstants.smallDriveVelocity)
            .withVelocityY(0)).withTimeout(AutonConstants.autonSmallDriveTimeoutSeconds);
    }


    public Command autonSmallRDriveCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive)
    {
        // timeSmall.restart();
        return drivetrain.applyRequest(() -> drive
            .withRotationalRate(0)
            .withVelocityX(AutonConstants.smallReverseDriveVelocity)
            .withVelocityY(0)).withTimeout(AutonConstants.autonSmallReverseDriveTimeoutSeconds);
    }

    // public boolean timeStop(double seconds)
    // {
    //     return timeSmall.get() >= seconds;
    // }

    public void setTagToBestTag() {
        // System.out.println("SSSSSSSSSSSSSSSSSSSSSS");
        if (camera.getLatestResult() == null) return;

        if (camera.getLatestResult().hasTargets()) {
            tagId = camera.getLatestResult().getBestTarget().getFiducialId();
        }
    }
    
    public RobotCentric zeroDrive(RobotCentric drive) {
       return drive
            .withRotationalRate(0)
            .withVelocityX(0)
            .withVelocityY(0);

    }

    public RobotCentric align(RobotCentric drive, LimelightAlignerDirection direction) {
        // System.out.println("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
        if (camera.getLatestResult() == null || !camera.getLatestResult().hasTargets())
        {
            // SmartDashboard.putBoolean("Zerod", true);
            return zeroDrive(drive);
        } else
        {
            double setpoint = 0;
            if (direction == LimelightAlignerDirection.Left)
                setpoint = -6.5;
            else
                setpoint = 6.5;

            //  double setpoint = FieldConstants.reefDistanceBetween / 2.0;
            // if (direction == LimelightAlignerDirection.Left) setpoint = -setpoint;
            // setpoint = setpoint - 1.75;

            return drive
                .withRotationalRate(-Math.signum(rotation.getDegrees())*rotationController.calculate(Math.abs(rotation.getDegrees()), 180))
                .withVelocityX(-0.5*relativeXController.calculate(distanceX, AutonConstants.limelightXSetpoint))
                .withVelocityY(-relativeYController.calculate(distanceY, Units.inchesToMeters(setpoint)));
        }
    }

    public RobotCentric autonAlign(RobotCentric drive, LimelightAlignerDirection direction) {
        if (camera.getLatestResult() == null || !camera.getLatestResult().hasTargets())
        {
            // SmartDashboard.putBoolean("Zerod", true);
            return zeroDrive(drive);
        } else
        {
            double setpoint = 0;
            if (direction == LimelightAlignerDirection.Left)
                setpoint = -6.5;
            else
                setpoint = 6.5;

            return drive
                .withRotationalRate(-Math.signum(rotation.getDegrees())*rotationController.calculate(Math.abs(rotation.getDegrees()), 180))
                .withVelocityX(-0.5*relativeXController.calculate(distanceX, AutonConstants.limelightXSetpoint))
                .withVelocityY(-relativeYController.calculate(distanceY, Units.inchesToMeters(setpoint)));
        }
    }
    // TODO should LimelightAligner be added as a requirement?
    public Command alignCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, LimelightAlignerDirection direction) {
        // return runOnce(() -> drivetrain.setCurrentLimit(Constants.TunerConstants.limelightCurrent))
        return runOnce(() -> drivetrain.setDriveLimit(50))
        .andThen(runOnce(() -> drivetrain.setSteerLimit(40)))
        .andThen(drivetrain.applyRequest(() -> align(drive, direction)).until(() -> isFinishedAlign()))
        .andThen(runOnce(() -> drivetrain.setDefaultLimits()));
    }

    public Command autonAlignCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, LimelightAlignerDirection direction) {
        return runOnce(() -> drivetrain.setCurrentLimit(Constants.TunerConstants.autonLimelightCurrent)).andThen(
        drivetrain.applyRequest(() -> autonAlign(drive, direction)).until(() -> isFinishedAlignAuton()))
        .andThen(runOnce(() -> drivetrain.setDefaultLimits()));
    }

    public Command autonAlignCommandSlow(CommandSwerveDrivetrain drivetrain, RobotCentric drive, LimelightAlignerDirection direction) {
        return runOnce(() -> drivetrain.setCurrentLimit(Constants.TunerConstants.autonLimelightCurrentSlow)).andThen(
        drivetrain.applyRequest(() -> autonAlign(drive, direction)).until(() -> isFinishedAlignAuton()))
        .andThen(runOnce(() -> drivetrain.setDefaultLimits()));
    }


    public RobotCentric autoRotate(CommandSwerveDrivetrain drivetrain, RobotCentric drive, int idx) {
        double measurement = drivetrain.getState().Pose.getRotation().getDegrees();
        double setpoint = 0;

        Alliance mySide = DriverStation.getAlliance().orElse(Alliance.Red);
        if (mySide == Alliance.Red)
            setpoint = FieldConstants.reefFacesRed[idx].getRotation().getDegrees();
        else
            setpoint = FieldConstants.reefFacesBlue[idx].getRotation().getDegrees();

        double calculation = rotationController2.calculate(measurement, setpoint);
        return drive
            .withRotationalRate(calculation);
    }

    public double setpointRotate(CommandSwerveDrivetrain drivetrain, boolean hasCoral) {
        setRot(drivetrain, hasCoral);
        Pose2d currentPose = drivetrain.getState().Pose;
        double measurement = currentPose.getRotation().getDegrees();
        double setpoint = targetPose.getRotation().getDegrees();

        // Alliance mySide = DriverStation.getAlliance().orElse(Alliance.Red);
        // if (mySide == Alliance.Red)
        //     setpoint = FieldConstants.reefFacesRed[idx].getRotation().getDegrees();
        // else
        //     setpoint = FieldConstants.reefFacesBlue[idx].getRotation().getDegrees();

        double calculation = rotationController3.calculate(measurement, setpoint);

        if (Math.abs(setpoint  - measurement) < rotationController3.getErrorTolerance() || targetPose.getTranslation().getDistance(currentPose.getTranslation()) > 1.5)
        {
            calculation = 0;
        }

        SmartDashboard.putNumber("rot", calculation);
        SmartDashboard.putNumber("rot2", targetPose.getTranslation().getDistance(currentPose.getTranslation()));


        return calculation;
    }

    public Command autoRotateCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, IntSupplier idxSupplier)
    {
        return drivetrain.applyRequest(() -> autoRotate(drivetrain, drive, idxSupplier.getAsInt())).until(this::isFinishedRot);
    }
    public Command autoRotateCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, int idx)
    {
        return drivetrain.applyRequest(() -> autoRotate(drivetrain, drive, idx)).until(this::isFinishedRot);
    }
    public Command autoRotateTeleopCommand(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, IntSupplier idxSupplier, DoubleSupplier x, DoubleSupplier y, double maxSpeed)
    {
        return drivetrain.applyRequest(() -> autoRotateTeleop(drivetrain, drive, idxSupplier.getAsInt(), x.getAsDouble(), y.getAsDouble(), maxSpeed)).until(this::isFinishedRot);
    }

    public SwerveRequest.FieldCentric autoRotateTeleop(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive, int idx, double x, double y, double maxSpeed) {
        double measurement = drivetrain.getState().Pose.getRotation().getDegrees();
        double setpoint = 0;
        if (side == Alliance.Red)
            setpoint = FieldConstants.reefFacesRed[idx].getRotation().getDegrees();
        else
            setpoint = FieldConstants.reefFacesBlue[idx].getRotation().getDegrees();
        double calculation = rotationController2.calculate(measurement, setpoint);

        return drive
            .withRotationalRate(calculation)
            .withVelocityX(-x*maxSpeed)
            .withVelocityY(-y*maxSpeed);
    }

    public boolean isFinishedRot()
    {
        return rotationController2.atSetpoint();
    }

    public boolean getManualTurn()
    {
        return manualTurn;
    }

    public void toggleTurn()
    {
        manualTurn = !manualTurn;
    }

    public void setTurn(boolean val)
    {
        manualTurn = val;
    }

    public boolean isFinishedAlign()
    {
        // (!latestResult.hasTargets() || latestResult == null)
        return (relativeXController.atSetpoint() && relativeYController.atSetpoint() && rotationController.atSetpoint()) || (camera.getLatestResult() == null || !camera.getLatestResult().hasTargets());
        //return relativeXController.atSetpoint() && relativeYController.atSetpoint() && rotationController.atSetpoint();
    }

    public boolean isFinishedAlignAuton()
    {
        boolean atSetpoint = relativeXController.atSetpoint() && relativeYController.atSetpoint() && rotationController.atSetpoint();
        // (!latestResult.hasTargets() || latestResult == null)
        // if (Math.abs(distanceX) <= VisionConstants.limelightMaxDistance)
        //     return atSetpoint || (camera.getLatestResult() == null || !camera.getLatestResult().hasTargets());
        // else
        //return atSetpoint;
        return (relativeXController.atSetpoint() && relativeYController.atSetpoint() && rotationController.atSetpoint()) || (camera.getLatestResult() == null || !camera.getLatestResult().hasTargets());

    }

    public Command autonCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, LimelightAlignerDirection direction, Reefscape reefscape) {
        return runOnce(() -> setTagToBestTag())
            .andThen(autonAlignCommand(drivetrain, drive, direction))
            .alongWith(reefscape.applyStateCommand(ReefscapeState.Level4, true, true, false))
            .andThen(autonSmallDriveCommand(drivetrain, drive))
            .andThen(reefscape.autonLevelCommand().withTimeout(0.3))
            .andThen(reefscape.zeroCommand())
            .andThen(autonSmallRDriveCommand(drivetrain, drive))
            .andThen(reefscape.applyStateCommand(ReefscapeState.Home, true, true, false));

    }

     public Command autonRCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, LimelightAlignerDirection direction, Reefscape reefscape, int idx) {
        return autoRotateCommand(drivetrain, drive, idx)
            .andThen(runOnce(() -> setTagToBestTag()))
            .andThen(autonAlignCommand(drivetrain, drive, direction))
            .alongWith(autonScoringSequence(drivetrain, drive, direction, reefscape))
            .andThen(autonSmallDriveCommand(drivetrain, drive))
            .andThen(autonDunkingCommand(drivetrain, drive, reefscape))
            .andThen(autonHomeSequence(drivetrain, drive, reefscape));
    }

    public Command autonRCommandSlow(CommandSwerveDrivetrain drivetrain, RobotCentric drive, LimelightAlignerDirection direction, Reefscape reefscape, int idx) {
        return autoRotateCommand(drivetrain, drive, idx)
            .andThen(runOnce(() -> setTagToBestTag()))
            .andThen(autonAlignCommandSlow(drivetrain, drive, direction))
            .alongWith(autonScoringSequence(drivetrain, drive, direction, reefscape))
            .andThen(autonSmallDriveCommand(drivetrain, drive))
            .andThen(autonDunkingCommand(drivetrain, drive, reefscape))
            .andThen(autonSmallRDriveCommand(drivetrain, drive))
            .andThen(autonHomeSequence(drivetrain, drive, reefscape));
    }

    public Command autonHomeSequence(CommandSwerveDrivetrain drivetrain, RobotCentric drive, Reefscape reefscape)
    {
        return reefscape.applyStateCommand(ReefscapeState.Home, true, false, false)
            .alongWith(new WaitCommand(0.38))
            .andThen(reefscape.applyStateCommand(ReefscapeState.Home, false, true, false));
    }

    
    public Command autonClearSequence(CommandSwerveDrivetrain drivetrain, RobotCentric drive, Reefscape reefscape)
    {
        return reefscape.applyStateCommand(ReefscapeState.Home, true, false, false)
            .alongWith(new WaitCommand(0.5))
            .andThen(reefscape.applyStateCommand(ReefscapeState.Clear, false, true, false));
    }

    public Command autonScoringSequence(CommandSwerveDrivetrain drivetrain, RobotCentric drive, LimelightAlignerDirection direction, Reefscape reefscape)
    {
        Command cmd = reefscape.applyStateCommandManual(() -> ReefscapeState.Clear, false, true, false)
            .alongWith(
                Commands.idle().until(() -> distanceX <= VisionConstants.autonLimelightElevatorDistance)
                    .andThen(reefscape.applyStateCommand(() -> ReefscapeState.Level4, true, false, false)));
        return cmd;
    }

    public Command autonDunkingCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, Reefscape reefscape)
    {
        return reefscape.applyStateCommand(ReefscapeState.Level4, false, true, false)
        .andThen(reefscape.autonLevelCommand().withTimeout(VisionConstants.autonDunkingTime))
        .andThen(reefscape.applyStateCommandManual(ReefscapeState.Clear, false, true, false))
        .andThen(reefscape.zeroCommand());
    }

    public void setRot(CommandSwerveDrivetrain drivetrain, boolean hasCoral)
    {
        Alliance color = DriverStation.getAlliance().orElse(Alliance.Red);
        Pose2d closestPose = new Pose2d();
        Pose2d currentPose = drivetrain.getState().Pose;
        double closestDistance = 0;
        double currentDistance = 0;
        if (hasCoral)
        {
            closestPose = Constants.FieldConstants.getReefPose(color, 0);
            closestDistance = currentPose.getTranslation().getDistance(closestPose.getTranslation());

            for (int i = 1; i <= 5; i++)
            {
                currentDistance = currentPose.getTranslation().getDistance(Constants.FieldConstants.getReefPose(color, i).getTranslation());
                if (currentDistance < closestDistance)
                {
                    closestPose = Constants.FieldConstants.getReefPose(color, i);
                    closestDistance = currentDistance;
                }
            }
        }
        else
        {
            closestPose = Constants.FieldConstants.getReefPose(color, 6);
            closestDistance = currentPose.getTranslation().getDistance(closestPose.getTranslation());
            currentDistance = currentPose.getTranslation().getDistance(Constants.FieldConstants.getReefPose(color, 7).getTranslation());
            if (currentDistance < closestDistance)
            {
                closestPose = Constants.FieldConstants.getReefPose(color, 7);
                closestDistance = currentDistance;
            }

        }

        targetPose = closestPose;
    }

    private PIDController makePIDFromConstants(PIDConstants constants) {
        return new PIDController(constants.kP, constants.kI, constants.kD);
    }
}
