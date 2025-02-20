package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.math.geometry.*;
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
        Manual;

        public boolean isLevel() {
            switch (this) {
                case Level1:
                case Level2:
                case Level3:
                case Level4:
                    return true;
                default:
                    return false;
            }
        }
    }

    public class PivotConstants {
        // TODO CHANGE THESE!!!
        public static final int motorId  = 17;
        public static final PIDConstants motorPidConstants  = new PIDConstants(1.0, 0.0, 0.0);
        public static final double motorkG = 1.0;
        public static final int motorEncoderAChannel   = 2;
        public static final int motorEncoderBChannel   = 3;
        public static final int motorEncoderPWMChannel = 4;


        public static final double[] anglesDegrees = {
            1234.0, // home
            1525.0, // coral station
            4926.0, // level 1
            3824.0, // level 2
            3824.0, // level 3
            3952.0, // level 4
        };
    }

    // All intake constants are in inches
    public class IntakeConstants {
        // TODO CHANGE THESE!!!
        public static final int motorId = 16;
        public static final PIDConstants motorPidConstants = new PIDConstants(1.0, 0.0, 0.0);
        public static final int innerProximitySensorChannel = 1;
        public static final int outerProximitySensorChannel = 2;

        public static final double motorRotationsPerWheelRotation = 12.0;
        public static final double wheelRadiusInches = 3.0;
        public static final double inchesPerMotorRotation = wheelRadiusInches / motorRotationsPerWheelRotation;

        public static final double intakeVelocity  =  10.0;
        public static final double outtakeVelocity = -10.0;
        public static final double[] velocitiesInchesPerSecond = {
            0.0,             // home
            intakeVelocity,  // coral station
            outtakeVelocity, // level 1
            outtakeVelocity, // level 2
            outtakeVelocity, // level 3
            intakeVelocity,  // level 4 (for level 4 we just intake in the other direction to outtake)
        };

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

        public static final double[] levelsInches = {
            0.0,  // home
            0.0,  // coral
            0.0,  // level 1
            0.0,  // level 2
            6.0,  // level 3
            20.0, // level 4
        };
    }

    public enum LimelightAlignerDirection {
        Left,
        Right,
    }

    public class VisionConstants {
        public record CameraWrapperConstants(String name, Transform3d robotToCamera) {};
        public static final CameraWrapperConstants[] camConstants = {
            new CameraWrapperConstants("ArducamFront",
                new Transform3d(new Translation3d(0*0.0254, -10.5*0.0254, 12.5*0.0254), new Rotation3d(0,0,Math.PI))),
            new CameraWrapperConstants("ArducamBack",
                new Transform3d(new Translation3d(9.5*0.0254, -10*0.0254, 18*0.0254), new Rotation3d(0,0,Math.PI/2))),
            new CameraWrapperConstants("ArducamLeft",
                new Transform3d(new Translation3d(-8.5*0.0254, 11.5*0.0254, 10*0.0254), new Rotation3d(0,0,0))),
            new CameraWrapperConstants("ArducamRight",
                new Transform3d(new Translation3d(-8.25*0.0254, -4*0.0254, 13.25*0.0254), new Rotation3d(0,0,3*Math.PI/2))),
            new CameraWrapperConstants("limelight",
                 new Transform3d(new Translation3d(0*0.0254, -11*0.0254, 11.5*0.0254), new Rotation3d(0,0,Math.PI)))
        };
        public static final double kalmanPositionStdDev = 1.2;
        public static final double kalmanRotationStdDev = 0.09;
        public static final PIDConstants limelightRotationPIDConstants  = new PIDConstants(0.07, 0, 0);
        public static final PIDConstants limelightRelativeXPIDConstants = new PIDConstants(1.8 /* 1.5 */);
        public static final PIDConstants limelightRelativeYPIDConstants = new PIDConstants(1 /* 1.5 */);
    }

    public class AutonConstants {
        public static final PIDConstants pathplannerTranslationPIDConstants = new PIDConstants(20, 0, 0);
        public static final PIDConstants pathplannerRotationPIDConstants    = new PIDConstants(9, 0, 0);
    }

    public class FieldConstants {
        public static final Pose2d[] reefFaces = new Pose2d[8]; // Starting facing the driver station in clockwise order

        static {
            // Initialize faces
            reefFaces[0] = new Pose2d(
                    Units.inchesToMeters(144.003),
                    Units.inchesToMeters(158.500),
                    Rotation2d.fromDegrees(180));
            reefFaces[1] = new Pose2d(
                    Units.inchesToMeters(160.373),
                    Units.inchesToMeters(186.857),
                    // Rotation2d.fromDegrees(180+60));
                    Rotation2d.fromDegrees(120));
            reefFaces[2] = new Pose2d(
                    Units.inchesToMeters(193.116),
                    Units.inchesToMeters(186.858),
                    Rotation2d.fromDegrees(60));
            // Rotation2d.fromDegrees(180+120));
            reefFaces[3] = new Pose2d(
                    Units.inchesToMeters(209.489),
                    Units.inchesToMeters(158.502),
                    Rotation2d.fromDegrees(0));
            reefFaces[4] = new Pose2d(
                    Units.inchesToMeters(193.118),
                    Units.inchesToMeters(130.145),
                    Rotation2d.fromDegrees(-60));
            reefFaces[5] = new Pose2d(
                    Units.inchesToMeters(160.375),
                    Units.inchesToMeters(130.144),
                    Rotation2d.fromDegrees(-120));
            reefFaces[6] = new Pose2d(
                    Units.inchesToMeters(33.526),
                    Units.inchesToMeters(291.176),
                    Rotation2d.fromDegrees(90 - 144.011));
            reefFaces[7] = new Pose2d(
                    Units.inchesToMeters(33.526),
                    Units.inchesToMeters(25.824),
                    Rotation2d.fromDegrees(144.011 - 90));

            Transform2d reefFaceTransform = new Transform2d(new Translation2d(1, 0.0), Rotation2d.kZero);
            Transform2d coralStationTransform = new Transform2d(new Translation2d(0.5, 0.0), Rotation2d.kZero);
            for (int i = 0; i < 6; i++) {
                reefFaces[i] = reefFaces[i].transformBy(reefFaceTransform);
            }
            for (int i = 6; i < reefFaces.length; i++) {
                reefFaces[i] = reefFaces[i].transformBy(coralStationTransform);
            }
        }

        public static final double coralStationToRobotThreshold = 1.0;
        public static final double reefDistanceBetween = Units.inchesToMeters(12.94);

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

        // The steer motor uses any SwerveModule.SteerRequestType control request with the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(0).withKD(0.5)
            .withKS(0.1).withKV(0).withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(0.1).withKI(0).withKD(0)
            .withKS(0).withKV(0.124);

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
        private static final Current kSlipCurrent = Amps.of(80);

        // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
        // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    // Swerve azimuth does not require much torque output, so we can set a relatively low
                    // stator current limit to help avoid brownouts without impacting performance.
                    .withStatorCurrentLimit(Amps.of(20))
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
        private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.28173828125);
        private static final boolean kFrontLeftSteerMotorInverted = false;
        private static final boolean kFrontLeftEncoderInverted = false;

        private static final Distance kFrontLeftXPos = Inches.of(13.5);
        private static final Distance kFrontLeftYPos = Inches.of(13.5);

        // Front Right
        private static final int kFrontRightDriveMotorId = 3;
        private static final int kFrontRightSteerMotorId = 7;
        private static final int kFrontRightEncoderId = 11;
        private static final Angle kFrontRightEncoderOffset = Rotations.of(0.2763671875);
        private static final boolean kFrontRightSteerMotorInverted = false;
        private static final boolean kFrontRightEncoderInverted = false;

        private static final Distance kFrontRightXPos = Inches.of(13.5);
        private static final Distance kFrontRightYPos = Inches.of(-13.5);

        // Back Left
        private static final int kBackLeftDriveMotorId = 2;
        private static final int kBackLeftSteerMotorId = 6;
        private static final int kBackLeftEncoderId = 10;
        private static final Angle kBackLeftEncoderOffset = Rotations.of(0.078125);
        private static final boolean kBackLeftSteerMotorInverted = false;
        private static final boolean kBackLeftEncoderInverted = false;

        private static final Distance kBackLeftXPos = Inches.of(-13.5);
        private static final Distance kBackLeftYPos = Inches.of(13.5);

        // Back Right
        private static final int kBackRightDriveMotorId = 4;
        private static final int kBackRightSteerMotorId = 8;
        private static final int kBackRightEncoderId = 12;
        private static final Angle kBackRightEncoderOffset = Rotations.of(-0.455810546875);
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
