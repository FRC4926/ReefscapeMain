package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Constants {
    public static enum ReefscapeState {
        Home,
        CoralStation,
        Level1,
        Level2,
        Level3,
        Level4,
        DeAlgaeL2,
        DeAlgaeL3,
        AlgaeL2,
        AlgaeL3,
        Barge,
        Clear,
        Manual;

        public boolean isLevel() {
            switch (this) {
                case Level1:
                    return true;
                case Level2:
                    return true;
                case Level3:
                    return true;
                case Level4:
                    return true;
                default:
                    return false;
            }
        }
    }

    public class ClimberConstants {
        public static final int climb1Id = 18;
        public static final int climb2Id = 19;

        public static final double climbForwardCurrentLimit = 10;
        public static final double climbBackCurrentLimit = 45;

        public static final double climberRatio = 340;

        public static final double climbVoltageForward = 0.5;
        public static final double climbVoltageBack = -0.85;

        public static final double climbThres = 129;
    }

    public class PivotConstants {
        // TODO CHANGE THESE!!!
        public static final int motorId = 17;
        public static final PIDConstants motorPidConstants  = new PIDConstants(1.8, 0.0, 0.0);
        public static final double motorkG = 1.0;
        public static final int motorEncoderAChannel   = 2;
        public static final int motorEncoderBChannel   = 3;
        public static final int motorEncoderPWMChannel = 4;
        public static final double currentLimit = 20;


        public static final double[] anglesDegrees = {
            10, // home
            25, // coral station
            10, // level 1
            117, // level 2
            123, // level 3
            98, // level 4
            51, // dealgae L2
            51, // dealgae L3
            0.0, //algae L2
            0.0, //algae L3
            0.0, //barge
            135 //clear
        };
    }

    
    public class AlgaeConstants {
        // TODO CHANGE THESE!!!
        public static final int motorId = 20;
        public static final PIDConstants motorPidConstants  = new PIDConstants(0.0, 0.0, 0.0);
        public static final double motorkG = 1.0;
        public static final double currentLimit = 20;
        public static final double gearRatio = 100;


        public static final double[] anglesDegrees = {
            0, // home
            0, // coral station
            0, // level 1
            0, // level 2
            0, // level 3
            0, // level 4
            0, // dealgae L2
            0, // dealgae l3
            0, // algae L2
            0, //algae L3
            0, //barge
            0 //clear
        };
    }

    public class AlgaeIntakeConstants {
        // TODO CHANGE THESE!!!
        public static final int motorId = 21;
        // public static final PIDConstants motorPidConstants = new PIDConstants(1.0, 0.0, 0.0);

        public static final double motorRotationsPerWheelRotation = 128.0;
        public static final double wheelRadiusInches = 3.0;
        public static final double inchesPerMotorRotation = wheelRadiusInches / motorRotationsPerWheelRotation;

        public static final double intakeVelocity  =  0.7;
        public static final double outtakeVelocity = -0.55;

        //public static final double autonIntakeVelocity  =  0.8;

        public static final int currentLimit = 30;

    }

    // All intake constants are in inches
    public class IntakeConstants {
        // TODO CHANGE THESE!!!
        public static final int motorId = 16;
        // public static final PIDConstants motorPidConstants = new PIDConstants(1.0, 0.0, 0.0);
        public static final int innerProximitySensorChannel = 3;
        public static final double cliffSensorThreshold = 2.0;
        // public static final int outerProximitySensorChannel = 9;

        public static final double motorRotationsPerWheelRotation = 12.0;
        public static final double wheelRadiusInches = 3.0;
        public static final double inchesPerMotorRotation = wheelRadiusInches / motorRotationsPerWheelRotation;

        public static final double intakeVelocity  =  0.9;
        public static final double outtakeVelocity = -0.65;

        public static final double autonIntakeVelocity  =  0.8;

        public static final double lowerIntakeVelocity  =  0.75;
        public static final double lowerOuttakeVelocity = -0.5;

        public static final int currentLimit = 30;

        // public static final double[] velocitiesInchesPerSecond = {
        //     0.0,             // home
        //     intakeVelocity,  // coral station
        //     outtakeVelocity, // level 1
        //     outtakeVelocity, // level 2
        //     outtakeVelocity, // level 3
        //     intakeVelocity,  // level 4 (for level 4 we just intake in the other direction to outtake)
        //     0.0,
        //     0.0,
        //     0.0,             // algae L3
        //     0.0,             //  algae L2
        //     0.0,             // barge
        //     0.0
        // };

        // public static enum Velocity {
        //     Zero,
        //     Intake,
        //     Outtake,
        // }
        // public static final double[] velocitiesMetersPerSecond = {
        //     0.0, // zero
        //     123.434,  // intake
        //     -123.435, // outtake
        // };
    }

    // Elevator constants are defined in inches
    public class ElevatorConstants {
        public static final int leftMotorCanId  = 14;
        public static final int rightMotorCanId = 15;
        public static final PIDConstants motorPidConstants = new PIDConstants(1.05, 0.0, 0.0);
        public static final double motorkG = 1.0;

        public static final double minPositionInches = 0.0;
        public static final double maxPositionInches = 25.0;

        public static final double motorRotationsPerShaftRotations = 9.0;
        public static final double inchesPerShaftRotation = 5.5;
        public static final double inchesPerMotorRotation = inchesPerShaftRotation / motorRotationsPerShaftRotations;

        public static final double currentLimit = 20;

        public static final double[] levelsInches = {
            0.0,  // home
            7.0,  // coral //6.8
            0.0,  // level 1
            0.0,  // level 2
            7.0,  // level 3
            21.5, // level 4
            4,    // dealgae L2
            10.8, // dealgae L3
            0,    // algae L2
            0,    // algae L3
            0,    // barge
            0
        };
    }

    public class LEDConstants {
        public static final int port = 0;
        public static final int numLeds = 60;
    }

    public enum LimelightAlignerDirection {
        Left,
        Right,
    }

    public class VisionConstants {
        public record CameraWrapperConstants(String name, Transform3d robotToCamera, double trustFactor) {};
        public static final CameraWrapperConstants[] camConstants = {
            new CameraWrapperConstants("ArducamFront",
                new Transform3d(new Translation3d(11.804*0.0254, 4.828*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,Math.PI/3)), 1),
            new CameraWrapperConstants("ArducamBack",
                new Transform3d(new Translation3d(11.804*0.0254, 2.184*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,2*Math.PI/3)), 1),
            new CameraWrapperConstants("ArducamLeft",
                new Transform3d(new Translation3d(-11.804*0.0254, -2.184*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,-2*Math.PI/3)), 1.25),
            new CameraWrapperConstants("ArducamRight",
                new Transform3d(new Translation3d(-11.804*0.0254, -4.828*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,-Math.PI/3)), 1),
            new CameraWrapperConstants("limelight",
                 new Transform3d(new Translation3d(0, 7*0.0254, 10.5*0.0254), new Rotation3d(0,-15*Math.PI/18,Math.PI)), 1.25)
        };
        public static final double kalmanPositionStdDevCoeefficient = 0.15;
        public static final double kalmanRotationStdDev = 0.99;

        public static final double maximumAmbiguity = 0.25;

        public static final PIDConstants limelightRotationPIDConstants  = new PIDConstants(0.07, 0, 0);
        public static final PIDConstants limelightRelativeXPIDConstants = new PIDConstants(6); // 6, 0, 0
        public static final PIDConstants limelightRelativeYPIDConstants = new PIDConstants(6, 0, 0.75); // 8, 0, 0.75

        public static final double limelightElevatorDistance = Units.inchesToMeters(33.5);
        public static final double autonLimelightElevatorDistance = Units.inchesToMeters(35.5);

        public static final double autonDunkingTime = 0.45;
        }

    public class AutonConstants {
        public static final PIDConstants pathplannerTranslationPIDConstants = new PIDConstants(3, 0, 0); //3, 10
        public static final PIDConstants pathplannerRotationPIDConstants    = new PIDConstants(5, 0, 0); //5, 18

        public static final PIDConstants flyTranslationPIDConstants = new PIDConstants(18, 0, 0);
        public static final PIDConstants flyRotationPIDConstants = new PIDConstants(10, 0, 0);

        public static final double smallDriveVelocity = 0.65;
        public static final double smallReverseDriveVelocity = -1.5;

        public static final double autonSmallDriveTimeoutSeconds = 0.3;  //1.5 works
        public static final double autonSmallReverseDriveTimeoutSeconds = 0.25;
        
        public static final double smallDriveTimeoutSeconds = 0.3;

        public static final double limelightXSetpoint = Units.inchesToMeters(6); //5.25
    }


    public class FieldConstants {
        public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        public static final double fieldWidth = Units.inchesToMeters(26*12 + 5);
        public static final double fieldHeight = Units.inchesToMeters(57*12 + 6 + 7.0/8.0);
        public static final Rectangle2d fieldRect = new Rectangle2d(new Translation2d(), new Translation2d(fieldHeight, fieldWidth));
        public static final Pose2d[] reefFacesRed = new Pose2d[8]; // Starting facing the driver station in clockwise order
        public static final Pose2d[] reefFacesBlue = new Pose2d[8]; // Starting facing the driver station in clockwise order

        static {
            // Initialize faces
            reefFacesRed[0] = new Pose2d(
                    Units.inchesToMeters(144.003),
                    Units.inchesToMeters(158.500),
                    Rotation2d.fromDegrees(180));
            reefFacesRed[1] = new Pose2d(
                    Units.inchesToMeters(160.373),
                    Units.inchesToMeters(186.857),
                    // Rotation2d.fromDegrees(180+60));
                    Rotation2d.fromDegrees(120));
            reefFacesRed[2] = new Pose2d(
                    Units.inchesToMeters(193.116),
                    Units.inchesToMeters(186.858),
                    Rotation2d.fromDegrees(60));
            // Rotation2d.fromDegrees(180+120));
            reefFacesRed[3] = new Pose2d(
                    Units.inchesToMeters(209.489),
                    Units.inchesToMeters(158.502),
                    Rotation2d.fromDegrees(0));
            reefFacesRed[4] = new Pose2d(
                    Units.inchesToMeters(193.118),
                    Units.inchesToMeters(130.145),
                    Rotation2d.fromDegrees(-60));
            reefFacesRed[5] = new Pose2d(
                    Units.inchesToMeters(160.375),
                    Units.inchesToMeters(130.144),
                    Rotation2d.fromDegrees(-120));
            reefFacesRed[6] = new Pose2d(
                    Units.inchesToMeters(33.526),
                    Units.inchesToMeters(291.176),
                    Rotation2d.fromDegrees(90 - 144.011));
            reefFacesRed[7] = new Pose2d(
                    Units.inchesToMeters(33.526),
                    Units.inchesToMeters(25.824),
                    Rotation2d.fromDegrees(144.011 - 90));

            Transform2d reefFaceTransform = new Transform2d(new Translation2d(-1.25, 0.0), Rotation2d.kZero);
            Transform2d coralStationTransform = new Transform2d(new Translation2d(-1, 0.0), Rotation2d.kZero);
            for (int i = 0; i < 6; i++) {
                reefFacesRed[i] = new Pose2d(reefFacesRed[i].getX(), reefFacesRed[i].getY(), reefFacesRed[i].getRotation().plus(Rotation2d.k180deg));
                reefFacesRed[i] = reefFacesRed[i].transformBy(reefFaceTransform);
            }
            for (int i = 6; i < reefFacesRed.length; i++) {
                reefFacesRed[i] = new Pose2d(reefFacesRed[i].getX(), reefFacesRed[i].getY(), reefFacesRed[i].getRotation().plus(Rotation2d.k180deg));
                reefFacesRed[i] = reefFacesRed[i].transformBy(coralStationTransform);
            }

            for (int i = 0; i < reefFacesBlue.length; i++) {
                reefFacesBlue[i] = new Pose2d(fieldHeight - reefFacesRed[i].getX(), fieldWidth - reefFacesRed[i].getY(), reefFacesRed[i].getRotation().rotateBy(Rotation2d.k180deg));
            }
        }

        public static Pose2d getReefPose(Alliance color, int id)
        {
            if (color == Alliance.Red)
            {
                return reefFacesRed[id];
            } else
            {
                return reefFacesBlue[id];
            }
        }

        public static final double coralStationToRobotThreshold = 1.0;
        public static final double reefDistanceBetween = Units.inchesToMeters(13);

        public static class CoralStation {
            public static final Pose2d leftCenterFace = new Pose2d(
                    Units.inchesToMeters(33.526),
                    Units.inchesToMeters(291.176),
                    Rotation2d.fromDegrees(90 - 144.011));
            public static final Pose2d rightCenterFace = new Pose2d(
                    Units.inchesToMeters(33.526),
                    Units.inchesToMeters(25.824),
                    Rotation2d.fromDegrees(144.011 - 90));
        }
    }

    // Generated by the Tuner X Swerve Project Generator
    // https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
    public class TunerConstants {
        // Both sets of gains need to be tuned to your individual robot.

        public static final double limelightCurrent = 30;
        public static final double autonLimelightCurrent = 40;
        public static final double autonLimelightCurrentSlow = 50;

        public static final double driveCurrent = 70;
        public static final double steerCurrent = 60;
        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(0).withKA(0) //kV = 2.66
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(10).withKS(5.5);
            // .withKP(0.1).withKI(0).withKD(0)
            // .withKS(0).withKV(0.124);

        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

        // The type of motor used for the drive motor
        private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
        // The type of motor used for the drive motor
        private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

        // The remote sensor feedback type to use for the steer motors;
        // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
        private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final Current kSlipCurrent = Amps.of(85);

        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(steerCurrent))
                    .withStatorCurrentLimitEnable(true)
            );
        private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
        // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
        private static final Pigeon2Configuration pigeonConfigs = null;

        // CAN bus that the devices are located on;
        // All swerve devices must share the same CAN bus
        public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

        // Theoretical free speed (m/s) at 12 V applied output;
        // This needs to be tuned to your individual robot
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.21);

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = 3.5714285714285716;

        private static final double kDriveGearRatio = 6.122448979591837;
        private static final double kSteerGearRatio = 12.8;
        private static final Distance kWheelRadius = Inches.of(2);

        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;

        private static final int kPigeonId = 13;

        // These are only used for simulation
        private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
        
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
        private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName(kCANBus.getName())
                .withPigeon2Id(kPigeonId)
                .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                .withDriveMotorType(kDriveMotorType)
                .withSteerMotorType(kSteerMotorType)
                .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);


        // Front Left
        private static final int kFrontLeftDriveMotorId = 1;
        private static final int kFrontLeftSteerMotorId = 5;
        private static final int kFrontLeftEncoderId = 9;
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.282958984375);
        private static final boolean kFrontLeftSteerMotorInverted = false;
        private static final boolean kFrontLeftEncoderInverted = false;

        private static final Distance kFrontLeftXPos = Inches.of(13.5);
        private static final Distance kFrontLeftYPos = Inches.of(13.5);

        // Front Right
        private static final int kFrontRightDriveMotorId = 3;
        private static final int kFrontRightSteerMotorId = 7;
        private static final int kFrontRightEncoderId = 11;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.008056640625);
        private static final boolean kFrontRightSteerMotorInverted = false;
        private static final boolean kFrontRightEncoderInverted = false;

        private static final Distance kFrontRightXPos = Inches.of(13.5);
        private static final Distance kFrontRightYPos = Inches.of(-13.5);

        // Back Left
        private static final int kBackLeftDriveMotorId = 2;
        private static final int kBackLeftSteerMotorId = 6;
        private static final int kBackLeftEncoderId = 10;
        private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.01708984375);
        private static final boolean kBackLeftSteerMotorInverted = false;
        private static final boolean kBackLeftEncoderInverted = false;

        private static final Distance kBackLeftXPos = Inches.of(-13.5);
        private static final Distance kBackLeftYPos = Inches.of(13.5);

        // Back Right
        private static final int kBackRightDriveMotorId = 4;
        private static final int kBackRightSteerMotorId = 8;
        private static final int kBackRightEncoderId = 12;
        private static final Angle kBackRightEncoderOffset = Rotations.of(-0.1845703125);
        private static final boolean kBackRightSteerMotorInverted = false;
        private static final boolean kBackRightEncoderInverted = false;

        private static final Distance kBackRightXPos = Inches.of(-13.5);
        private static final Distance kBackRightYPos = Inches.of(-13.5);


        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
                kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
                kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
                kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
            );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
                kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
            );

        /**
         * Creates a CommandSwerveDrivetrain instance.
         * This should only be called once in your robot program,.
         */
        public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            new Translation2d(kFrontLeftXPos, kFrontLeftYPos),
            new Translation2d(kFrontRightXPos, kFrontRightYPos),
            new Translation2d(kBackLeftXPos, kBackLeftYPos),
            new Translation2d(kBackRightXPos, kBackRightYPos)
        );
        public static CommandSwerveDrivetrain createDrivetrain() {
            return new CommandSwerveDrivetrain(
                DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
            );
        }


        /**
         * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
         */
        public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
             * @param modules               Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, modules
                );
            }

            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
             * @param odometryUpdateFrequency The frequency to run the odometry loop. If
             *                                unspecified or set to 0 Hz, this is 250 Hz on
             *                                CAN FD, and 100 Hz on CAN 2.0.
             * @param modules                 Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency, modules
                );
            }

            /**
             * Constructs a CTRE SwerveDrivetrain using the specified constants.
             * <p>
             * This constructs the underlying hardware devices, so users should not construct
             * the devices themselves. If they need the devices, they can access them through
             * getters in the classes.
             *
             * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
             * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
             *                                  unspecified or set to 0 Hz, this is 250 Hz on
             *                                  CAN FD, and 100 Hz on CAN 2.0.
             * @param odometryStandardDeviation The standard deviation for odometry calculation
             *                                  in the form [x, y, theta]ᵀ, with units in meters
             *                                  and radians
             * @param visionStandardDeviation   The standard deviation for vision calculation
             *                                  in the form [x, y, theta]ᵀ, with units in meters
             *                                  and radians
             * @param modules                   Constants for each specific module
             */
            public TunerSwerveDrivetrain(
                SwerveDrivetrainConstants drivetrainConstants,
                double odometryUpdateFrequency,
                Matrix<N3, N1> odometryStandardDeviation,
                Matrix<N3, N1> visionStandardDeviation,
                SwerveModuleConstants<?, ?, ?>... modules
            ) {
                super(
                    TalonFX::new, TalonFX::new, CANcoder::new,
                    drivetrainConstants, odometryUpdateFrequency,
                    odometryStandardDeviation, visionStandardDeviation, modules
                );
            }
        }
    }
}
