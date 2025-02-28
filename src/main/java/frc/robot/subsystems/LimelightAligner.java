package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.hal.PWMJNI;
import edu.wpi.first.hal.simulation.PWMDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightAlignerDirection;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraWrapperConstants;

public class LimelightAligner extends SubsystemBase {
    // private final PhotonCamera camera = new PhotonCamera("limelight");
    private final CameraWrapper camera = RobotContainer.visionSubsystem.getCameras().get(4);
    private int tagId = -1;
    private double yaw = 0.0;
    private double distanceX = 0.0;
    private double distanceY = 0.0;
    private double distanceZ = 0.0;
    private Rotation2d rotation = Rotation2d.kZero;
    private final PIDController rotationController  = makePIDFromConstants(VisionConstants.limelightRotationPIDConstants);
    private final PIDController rotationController2  = makePIDFromConstants(VisionConstants.limelightRotationPIDConstants);
    private final PIDController relativeXController = makePIDFromConstants(VisionConstants.limelightRelativeXPIDConstants);
    private final PIDController relativeYController = makePIDFromConstants(VisionConstants.limelightRelativeYPIDConstants);

    // Timer timeSmall = new Timer();

    public LimelightAligner() {
        rotationController.setTolerance(0.1);
        rotationController2.setTolerance(5);
        relativeXController.setTolerance(0.01);
        relativeYController.setTolerance(0.01);

        rotationController2.enableContinuousInput(-180, 180);
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
        SmartDashboard.putBoolean("Is fin", isFinishedRot());
        if (!camera.isConnected()) {
            SmartDashboard.putBoolean("COULD NOT CONNECT", true);
        }


        // List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (camera.getLatestResult() != null) {
            for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
                if (target.getFiducialId() == tagId) {
                    yaw = target.getYaw();
                    target.getBestCameraToTarget();
                    Transform3d camToTag = target.getBestCameraToTarget();
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

    public Command smallDriveCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive)
    {
        // timeSmall.restart();
        return drivetrain.applyRequest(() -> drive
        .withRotationalRate(0)
        .withVelocityX(0.75)
        .withVelocityY(0)).withTimeout(.5);
    }

    // public boolean timeStop(double seconds)
    // {
    //     return timeSmall.get() >= seconds;
    // }

    public void setTagToBestTag() {
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
        if (camera.getLatestResult() == null || !camera.getLatestResult().hasTargets())
        {
            SmartDashboard.putBoolean("Zerod", true);
            return zeroDrive(drive);
        } else
        {
            double setpoint = 0;
            if (direction == LimelightAlignerDirection.Left)
                setpoint = -6.5 - 1.75;
            else
                setpoint = 6.5 - 1.75;

            //  double setpoint = FieldConstants.reefDistanceBetween / 2.0;
            // if (direction == LimelightAlignerDirection.Left) setpoint = -setpoint;
            // setpoint = setpoint - 1.75;

            return drive
                .withRotationalRate(-Math.signum(rotation.getDegrees())*rotationController.calculate(Math.abs(rotation.getDegrees()), 180))
                .withVelocityX(-0.5*relativeXController.calculate(distanceX, 0))
                .withVelocityY(-relativeYController.calculate(distanceY, Units.inchesToMeters(setpoint)));
        }
    }
    // TODO should LimelightAligner be added as a requirement?
    public Command alignCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, LimelightAlignerDirection direction) {
        return drivetrain.applyRequest(() -> align(drive, direction)).until(() -> isFinishedAlign());
    }

    public RobotCentric autoRotate(CommandSwerveDrivetrain drivetrain, RobotCentric drive, int idx) {
        double measurement = drivetrain.getState().Pose.getRotation().getDegrees();
        double setpoint    = FieldConstants.reefFaces[idx].getRotation().getDegrees();
        double calculation = rotationController2.calculate(measurement, setpoint);
        return drive
            .withRotationalRate(calculation);
    }

    public Command autoRotateCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, IntSupplier idxSupplier)
    {
        return drivetrain.applyRequest(() -> autoRotate(drivetrain, drive, idxSupplier.getAsInt())).until(this::isFinishedRot);
    }
    public Command coralStationAlignCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, IntSupplier idxSupplier, DoubleSupplier x, DoubleSupplier y, double maxSpeed)
    {
        return drivetrain.applyRequest(() -> coralRotate(drivetrain, drive, idxSupplier.getAsInt(), x.getAsDouble(), y.getAsDouble(), maxSpeed));
    }

    public RobotCentric coralRotate(CommandSwerveDrivetrain drivetrain, RobotCentric drive, int idx, double x, double y, double maxSpeed) {
        double measurement = drivetrain.getState().Pose.getRotation().getDegrees();
        double setpoint    = FieldConstants.reefFaces[idx].getRotation().plus(Rotation2d.k180deg).getDegrees();
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

    public boolean isFinishedAlign()
    {
        //(!latestResult.hasTargets() || latestResult == null)
        return (relativeXController.atSetpoint() && relativeYController.atSetpoint() && rotationController.atSetpoint()) || (camera.getLatestResult() == null || !camera.getLatestResult().hasTargets());
    }

    public Command autonCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive, LimelightAlignerDirection direction) {
        return runOnce(() -> setTagToBestTag()).andThen(alignCommand(drivetrain, drive, direction));
    }

    private PIDController makePIDFromConstants(PIDConstants constants) {
        return new PIDController(constants.kP, constants.kI, constants.kD);
    }
}
