package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.RobotContainer;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    // private boolean interrupt = true;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    Command generatedPath = null;
    // Pose2d sheesh = null;
    private boolean inverted = false;

    
    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

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
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        init();
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
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        init();
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
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        init();
    }

    // Called across all `CommandSwerveDrivetrain` constructors
    public void init() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

        if (alliance == Alliance.Blue)
            inverted = true;
    }

    public double getTotalCurrent() {
        double ret = 0.0;
        for (var module : getModules()) {
            ret += module.getDriveMotor().getStatorCurrent().getValueAsDouble();
            ret += module.getSteerMotor().getStatorCurrent().getValueAsDouble();
        }

        return ret;
    }


    public void setCurrentLimit(double current)
    {
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(current);
        for (var module : getModules()) {
            module.getDriveMotor().getConfigurator().apply(currentLimitsConfigs);
            module.getSteerMotor().getConfigurator().apply(currentLimitsConfigs);
        }
    }

    private void configureAutoBuilder() {
        //System.out.println("CONFIGUREAUTOBUILDER CALLED!!!!!!!!!!");
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    AutonConstants.pathplannerTranslationPIDConstants,
                    // PID constants for rotation
                    AutonConstants.pathplannerRotationPIDConstants
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }


    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {

        // SmartDashboard.putBoolean("Interrupt", interrupt);
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    // public Command updatedPath() {
    //     return updatedPath(FieldConstants.reefFaces[RobotContainer.getReefFaceIdx()]);
    // }

    // public Command updatedPath(Pose2d targetPose)
    // {
    //     RobotContainer.targetPosePublisher.set(targetPose);
        
    //     PathConstraints constraints = new PathConstraints(
    //             1, 1,
    //             Units.degreesToRadians(540), Units.degreesToRadians(720));
    //     return generatedPath = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);//.until(() -> interrupt);

    // }

    public Command updatedPathCommand(Supplier<Integer> targetPoseSupplier) {
        // Pose2d targetPose = targetPoseSupplier.get();

        // RobotContainer.targetPosePublisher.set(targetPose);
        
        PathConstraints constraints = new PathConstraints(
                3, 2,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        // return generatedPath = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0).onlyWhile(() -> interrupt);

        // Commands.startRun()

        // generatedPath = runOnce(() -> {
        //     Command command = AutoBuilder.pathfindToPose(targetPoseSupplier.get(), constraints, 0.0)
        //         .alongWith(onStart)
        //         .andThen(onEnd);
        //     command.asProxy();
        //     command.addRequirements(this);
        //     command.schedule();
        // });
        //     // .onlyWhile(() -> interrupt);
        // return generatedPath;
        Pose2d setpointP = null;
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            setpointP = FieldConstants.reefFacesRed[targetPoseSupplier.get()];
        else
            setpointP = FieldConstants.reefFacesBlue[targetPoseSupplier.get()];
        final Pose2d setpointPF = setpointP;
        
        generatedPath = defer(() -> AutoBuilder.pathfindToPose(setpointPF, constraints, 0.0));
            //.onlyWhile(() -> interrupt);

        return generatedPath;
    }

    public Command updatedPath(Supplier<Integer> targetPoseSupplier) {
        Pose2d setpointP;
        if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red)
            setpointP = FieldConstants.reefFacesRed[targetPoseSupplier.get()];
        else
            setpointP = FieldConstants.reefFacesBlue[targetPoseSupplier.get()];
        return updatedPath(setpointP);
    }

    public Command updatedPath(Pose2d targetPose)
    {
        RobotContainer.targetPosePublisher.set(targetPose);
        
        PathConstraints constraints = new PathConstraints(
                3, 1,
                Units.degreesToRadians(540), Units.degreesToRadians(720));
        return generatedPath = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);

    }

    // public Command limelightUpdate()
    // {
    //     CameraWrapper limelight = RobotContainer.visionSubsystem.getCameras().get(4);
    //     Optional<EstimatedRobotPose> estimated = limelight.getEstimatedGlobalPose();
    //     EstimatedRobotPose limelightPose = estimated.isPresent() ? estimated.get() : null;

        
    //     PathConstraints constraints = new PathConstraints(
    //             3, 3,
    //             Units.degreesToRadians(540), Units.degreesToRadians(720));
    //     PathfindingCommand limePath = null;
    //     try
    //     {
    //     var config = RobotConfig.fromGUISettings();
    //     limePath = new PathfindingCommand(
    //         FieldConstants.reefFaces[limelight.getBestTarget().getFiducialId()],
    //         constraints,
    //         0,
    //         () -> limelightPose.estimatedPose.toPose2d(),
    //         () -> getState().Speeds,
    //         (speeds, feedforwards) -> setControl(
    //                 m_pathApplyRobotSpeeds.withSpeeds(speeds)
    //                     .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
    //                     .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
    //             ),
    //         new PPHolonomicDriveController(
    //             // PID constants for translation
    //             AutonConstants.pathplannerTranslationPIDConstants,
    //             // PID constants for rotation
    //             AutonConstants.pathplannerRotationPIDConstants
    //         ),
    //         config,
    //         this);
    //     } catch (Exception ex) {
    //         DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    //     }

    //     return limePath;

    // }

    // public void setInterrupt(boolean value)
    // {
    //     interrupt = value;
    // }

    // public Pose2d targetChange() {
    //     if(RobotContainer.logitechController.getRawButton(1)) {
    //         sheesh = FieldConstants.reefFaces[5];
    //     } else if(RobotContainer.logitechController.getRawButton(2)) {
    //         sheesh = FieldConstants.CoralStation.rightCenterFace;
    //     }

    //     return sheesh;
    // }

    public Command generatedPath()
    {
   
        return generatedPath;
    }

}
