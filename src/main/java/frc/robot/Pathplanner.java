package frc.robot;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilder.TriFunction;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class Pathplanner {
    // public static final AutoBuilder4926 auton = new AutoBuilder4926();
    private static TriFunction<Pose2d, PathConstraints, Double, Command> flyCommandBuilder;

    private static Supplier<Pose2d> poseSupplier;
    private static Consumer<Pose2d> resetPose;
    private static Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier;
    private static BiConsumer<ChassisSpeeds, DriveFeedforwards> output;
    private static Function<PathPlannerPath, Command> pathFollowingCommandBuilder;
    private static RobotConfig robotConfig;
    private static BooleanSupplier shouldFlipPath;
    private static boolean isHolonomic;

    private static PPHolonomicDriveController autonController;
    private static PPHolonomicDriveController flyController;

    private static boolean configured = false;

    public static void configure(CommandSwerveDrivetrain drivetrain) {
        if (configured) {
            System.out.println("Pathplanner.configure() has already been called!");
            return;
        }
        configured = true;

        poseSupplier = () -> drivetrain.getState().Pose;
        resetPose = drivetrain::resetPose;
        robotRelativeSpeedsSupplier = () -> drivetrain.getState().Speeds;
        output = (speeds, feedforwards) -> drivetrain.setControl(
            new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
        );
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            System.out.println("Could NOT get PathPlanner GUI settings\n" + e);
        }
        shouldFlipPath = () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue;

        autonController = new PPHolonomicDriveController(
            AutonConstants.pathplannerTranslationPIDConstants,
            AutonConstants.pathplannerRotationPIDConstants
        );
        flyController = new PPHolonomicDriveController(
            AutonConstants.flyTranslationPIDConstants,
            AutonConstants.flyRotationPIDConstants
        );
        

        AutoBuilder.configure(
            poseSupplier,
            resetPose,
            robotRelativeSpeedsSupplier,
            output,
            autonController,
            robotConfig,
            shouldFlipPath,
            drivetrain
        );

        flyCommandBuilder = (pose, constraints, goalEndVel) -> new PathfindingCommand(
            pose,
            constraints,
            goalEndVel,
            poseSupplier,
            robotRelativeSpeedsSupplier,
            output,
            flyController,
            robotConfig,
            drivetrain
        );
    }

    public static Command pathfindToPose(Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
        if (!configured) {
            System.out.println("Please call Pathplanner.configure() first.");
            return null;
        }
        return flyCommandBuilder.apply(pose, constraints, goalEndVelocity);
    }
}
