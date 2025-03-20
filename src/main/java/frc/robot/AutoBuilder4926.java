// package frc.robot;

// import edu.wpi.first.units.Units;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.*;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PathFollowingController;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.util.DriveFeedforwards;
// import com.pathplanner.lib.util.FlippingUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.units.measure.LinearVelocity;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.Subsystem;
// import java.io.File;
// import java.util.ArrayList;
// import java.util.List;
// import java.util.function.*;
// import java.util.stream.Collectors;
// import java.util.stream.Stream;

// /** Utility class used to build auto routines */
// public class AutoBuilder4926 {
//   private  boolean configured = false;

//   private  Supplier<Pose2d> poseSupplier;
//   private  Function<PathPlannerPath, Command> pathFollowingCommandBuilder;
//   private  Consumer<Pose2d> resetPose;
//   private  BooleanSupplier shouldFlipPath;
//   private  boolean isHolonomic;

//   // Pathfinding builders
//   private  boolean pathfindingConfigured = false;
//   private  TriFunction<Pose2d, PathConstraints, Double, Command> pathfindToPoseCommandBuilder;
//   private  BiFunction<PathPlannerPath, PathConstraints, Command>
//       pathfindThenFollowPathCommandBuilder;

//   /**
//    * Configures the this for using PathPlanner's built-in commands.
//    *
//    * @param poseSupplier a supplier for the robot's current pose
//    * @param resetPose a consumer for resetting the robot's pose
//    * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
//    *     speeds
//    * @param output Output function that accepts robot-relative ChassisSpeeds and feedforwards for
//    *     each drive motor. If using swerve, these feedforwards will be in FL, FR, BL, BR order. If
//    *     using a differential drive, they will be in L, R order.
//    *     <p>NOTE: These feedforwards are assuming unoptimized module states. When you optimize your
//    *     module states, you will need to reverse the feedforwards for modules that have been flipped
//    * @param controller Path following controller that will be used to follow paths
//    * @param robotConfig The robot configuration
//    * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side of
//    *     the field. This will maintain a global blue alliance origin.
//    * @param driveRequirements the subsystem requirements for the robot's drive train
//    */
//   public  void configure(
//       Supplier<Pose2d> poseSupplier,
//       Consumer<Pose2d> resetPose,
//       Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
//       BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
//       PathFollowingController controller,
//       RobotConfig robotConfig,
//       BooleanSupplier shouldFlipPath,
//       Subsystem... driveRequirements) {
//     if (configured) {
//       DriverStation.reportError(
//           "Auto builder has already been configured. This is likely in error.", true);
//     }

//     this.pathFollowingCommandBuilder =
//         (path) ->
//             new FollowPathCommand(
//                 path,
//                 poseSupplier,
//                 robotRelativeSpeedsSupplier,
//                 output,
//                 controller,
//                 robotConfig,
//                 shouldFlipPath,
//                 driveRequirements);
//     this.poseSupplier = poseSupplier;
//     this.resetPose = resetPose;
//     this.configured = true;
//     this.shouldFlipPath = shouldFlipPath;
//     this.isHolonomic = robotConfig.isHolonomic;

//     this.pathfindToPoseCommandBuilder =
//         (pose, constraints, goalEndVel) ->
//             new PathfindingCommand(
//                 pose,
//                 constraints,
//                 goalEndVel,
//                 poseSupplier,
//                 robotRelativeSpeedsSupplier,
//                 output,
//                 controller,
//                 robotConfig,
//                 driveRequirements);
//     this.pathfindThenFollowPathCommandBuilder =
//         (path, constraints) ->
//             new PathfindThenFollowPath(
//                 path,
//                 constraints,
//                 poseSupplier,
//                 robotRelativeSpeedsSupplier,
//                 output,
//                 controller,
//                 robotConfig,
//                 shouldFlipPath,
//                 driveRequirements);
//     this.pathfindingConfigured = true;
//   }

//   /**
//    * Configures the this for using PathPlanner's built-in commands.
//    *
//    * @param poseSupplier a supplier for the robot's current pose
//    * @param resetPose a consumer for resetting the robot's pose
//    * @param robotRelativeSpeedsSupplier a supplier for the robot's current robot relative chassis
//    *     speeds
//    * @param output Output function that accepts robot-relative ChassisSpeeds.
//    * @param controller Path following controller that will be used to follow paths
//    * @param robotConfig The robot configuration
//    * @param shouldFlipPath Supplier that determines if paths should be flipped to the other side of
//    *     the field. This will maintain a global blue alliance origin.
//    * @param driveRequirements the subsystem requirements for the robot's drive train
//    */
//   public  void configure(
//       Supplier<Pose2d> poseSupplier,
//       Consumer<Pose2d> resetPose,
//       Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
//       Consumer<ChassisSpeeds> output,
//       PathFollowingController controller,
//       RobotConfig robotConfig,
//       BooleanSupplier shouldFlipPath,
//       Subsystem... driveRequirements) {
//     configure(
//         poseSupplier,
//         resetPose,
//         robotRelativeSpeedsSupplier,
//         (speeds, feedforwards) -> output.accept(speeds),
//         controller,
//         robotConfig,
//         shouldFlipPath,
//         driveRequirements);
//   }

//   /**
//    * Configures the this with custom path following command builder. Building pathfinding
//    * commands is not supported if using a custom command builder. Custom path following commands
//    * will not have the path flipped for them, and event markers will not be triggered automatically.
//    *
//    * @param pathFollowingCommandBuilder a function that builds a command to follow a given path
//    * @param poseSupplier a supplier for the robot's current pose
//    * @param resetPose a consumer for resetting the robot's pose
//    * @param shouldFlipPose Supplier that determines if the starting pose should be flipped to the
//    *     other side of the field. This will maintain a global blue alliance origin. NOTE: paths will
//    *     not be flipped when configured with a custom path following command. Flipping the paths
//    *     must be handled in your command.
//    * @param isHolonomic Does the robot have a holonomic drivetrain
//    */
//   public  void configureCustom(
//       Function<PathPlannerPath, Command> pathFollowingCommandBuilder,
//       Supplier<Pose2d> poseSupplier,
//       Consumer<Pose2d> resetPose,
//       BooleanSupplier shouldFlipPose,
//       boolean isHolonomic) {
//     if (configured) {
//       DriverStation.reportError(
//           "Auto builder has already been configured. This is likely in error.", true);
//     }

//     this.pathFollowingCommandBuilder = pathFollowingCommandBuilder;
//     this.poseSupplier = poseSupplier;
//     this.resetPose = resetPose;
//     this.configured = true;
//     this.shouldFlipPath = shouldFlipPose;
//     this.isHolonomic = isHolonomic;

//     this.pathfindingConfigured = false;
//   }

//   /**
//    * Configures the this with custom path following command builder. Building pathfinding
//    * commands is not supported if using a custom command builder. Custom path following commands
//    * will not have the path flipped for them, and event markers will not be triggered automatically.
//    *
//    * @param pathFollowingCommandBuilder a function that builds a command to follow a given path
//    * @param poseSupplier a supplier for the robot's current pose
//    * @param resetPose a consumer for resetting the robot's pose
//    * @param isHolonomic Does the robot have a holonomic drivetrain
//    */
//   public  void configureCustom(
//       Function<PathPlannerPath, Command> pathFollowingCommandBuilder,
//       Supplier<Pose2d> poseSupplier,
//       Consumer<Pose2d> resetPose,
//       boolean isHolonomic) {
//     configureCustom(pathFollowingCommandBuilder, poseSupplier, resetPose, () -> false, isHolonomic);
//   }

//   /**
//    * Returns whether the this has been configured.
//    *
//    * @return true if the this has been configured, false otherwise
//    */
//   public  boolean isConfigured() {
//     return configured;
//   }

//   /**
//    * Returns whether the this has been configured for pathfinding.
//    *
//    * @return true if the this has been configured for pathfinding, false otherwise
//    */
//   public  boolean isPathfindingConfigured() {
//     return pathfindingConfigured;
//   }

//   /**
//    * Get the current robot pose
//    *
//    * @return Current robot pose
//    */
//   public  Pose2d getCurrentPose() {
//     return poseSupplier.get();
//   }

//   /**
//    * Get if a path or field position should currently be flipped
//    *
//    * @return True if path/positions should be flipped
//    */
//   public  boolean shouldFlip() {
//     return shouldFlipPath.getAsBoolean();
//   }

//   /**
//    * Builds a command to follow a path. PathPlannerLib commands will also trigger event markers
//    * along the way.
//    *
//    * @param path the path to follow
//    * @return a path following command with for the given path
//    * @throws Exception if the this has not been configured
//    */
//   public  Command followPath(PathPlannerPath path) {
//     if (!isConfigured()) {
//       System.out.println(
//           "Auto builder was used to build a path following command before being configured");
//     }

//     return pathFollowingCommandBuilder.apply(path);
//   }

//   /**
//    * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
//    * rotation and rotation delay distance will have no effect.
//    *
//    * @param pose The pose to pathfind to
//    * @param constraints The constraints to use while pathfinding
//    * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
//    * @return A command to pathfind to a given pose
//    */
//   public  Command pathfindToPose(
//       Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
//     if (!isPathfindingConfigured()) {
//       System.out.println(
//           "Auto builder was used to build a pathfinding command before being configured");
//     }

//     return pathfindToPoseCommandBuilder.apply(pose, constraints, goalEndVelocity);
//   }

//   /**
//    * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
//    * rotation and rotation delay distance will have no effect.
//    *
//    * @param pose The pose to pathfind to
//    * @param constraints The constraints to use while pathfinding
//    * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
//    * @return A command to pathfind to a given pose
//    */
//   public  Command pathfindToPose(
//       Pose2d pose, PathConstraints constraints, LinearVelocity goalEndVelocity) {
//     return pathfindToPose(pose, constraints, goalEndVelocity.in(Units.MetersPerSecond));
//   }

//   /**
//    * Build a command to pathfind to a given pose. If not using a holonomic drivetrain, the pose
//    * rotation will have no effect.
//    *
//    * @param pose The pose to pathfind to
//    * @param constraints The constraints to use while pathfinding
//    * @return A command to pathfind to a given pose
//    */
//   public  Command pathfindToPose(Pose2d pose, PathConstraints constraints) {
//     return pathfindToPose(pose, constraints, 0);
//   }

//   /**
//    * Build a command to pathfind to a given pose that will be flipped based on the value of the path
//    * flipping supplier when this command is run. If not using a holonomic drivetrain, the pose
//    * rotation and rotation delay distance will have no effect.
//    *
//    * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier returns
//    *     true
//    * @param constraints The constraints to use while pathfinding
//    * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
//    * @return A command to pathfind to a given pose
//    */
//   public  Command pathfindToPoseFlipped(
//       Pose2d pose, PathConstraints constraints, double goalEndVelocity) {
//     return Commands.either(
//         pathfindToPose(FlippingUtil.flipFieldPose(pose), constraints, goalEndVelocity),
//         pathfindToPose(pose, constraints, goalEndVelocity),
//         shouldFlipPath);
//   }

//   /**
//    * Build a command to pathfind to a given pose that will be flipped based on the value of the path
//    * flipping supplier when this command is run. If not using a holonomic drivetrain, the pose
//    * rotation and rotation delay distance will have no effect.
//    *
//    * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier returns
//    *     true
//    * @param constraints The constraints to use while pathfinding
//    * @param goalEndVelocity The goal end velocity of the robot when reaching the target pose
//    * @return A command to pathfind to a given pose
//    */
//   public  Command pathfindToPoseFlipped(
//       Pose2d pose, PathConstraints constraints, LinearVelocity goalEndVelocity) {
//     return pathfindToPoseFlipped(pose, constraints, goalEndVelocity.in(Units.MetersPerSecond));
//   }

//   /**
//    * Build a command to pathfind to a given pose that will be flipped based on the value of the path
//    * flipping supplier when this command is run. If not using a holonomic drivetrain, the pose
//    * rotation and rotation delay distance will have no effect.
//    *
//    * @param pose The pose to pathfind to. This will be flipped if the path flipping supplier returns
//    *     true
//    * @param constraints The constraints to use while pathfinding
//    * @return A command to pathfind to a given pose
//    */
//   public  Command pathfindToPoseFlipped(Pose2d pose, PathConstraints constraints) {
//     return pathfindToPoseFlipped(pose, constraints, 0);
//   }

//   /**
//    * Build a command to pathfind to a given path, then follow that path. If not using a holonomic
//    * drivetrain, the pose rotation delay distance will have no effect.
//    *
//    * @param goalPath The path to pathfind to, then follow
//    * @param pathfindingConstraints The constraints to use while pathfinding
//    * @return A command to pathfind to a given path, then follow the path
//    */
//   public  Command pathfindThenFollowPath(
//       PathPlannerPath goalPath, PathConstraints pathfindingConstraints) {
//     if (!isPathfindingConfigured()) {
//       System.out.println(
//           "Auto builder was used to build a pathfinding command before being configured");
//     }

//     return pathfindThenFollowPathCommandBuilder.apply(goalPath, pathfindingConstraints);
//   }

//   /**
//    * Create and populate a sendable chooser with all PathPlannerAutos in the project. The default
//    * option will be Commands.none()
//    *
//    * @return SendableChooser populated with all autos
//    */
//   public  SendableChooser<Command> buildAutoChooser() {
//     return buildAutoChooser("");
//   }

//   /**
//    * Create and populate a sendable chooser with all PathPlannerAutos in the project
//    *
//    * @param defaultAutoName The name of the auto that should be the default option. If this is an
//    *     empty string, or if an auto with the given name does not exist, the default option will be
//    *     Commands.none()
//    * @return SendableChooser populated with all autos
//    */
//   public  SendableChooser<Command> buildAutoChooser(String defaultAutoName) {
//     return buildAutoChooserWithOptionsModifier(defaultAutoName, (stream) -> stream);
//   }

//   /**
//    * Create and populate a sendable chooser with all PathPlannerAutos in the project. The default
//    * option will be Commands.none()
//    *
//    * @param optionsModifier A lambda function that can be used to modify the options before they go
//    *     into the AutoChooser
//    * @return SendableChooser populated with all autos
//    */
//   public  SendableChooser<Command> buildAutoChooserWithOptionsModifier(
//       Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
//     return buildAutoChooserWithOptionsModifier("", optionsModifier);
//   }

//   /**
//    * Create and populate a sendable chooser with all PathPlannerAutos in the project
//    *
//    * @param defaultAutoName The name of the auto that should be the default option. If this is an
//    *     empty string, or if an auto with the given name does not exist, the default option will be
//    *     Commands.none()
//    * @param optionsModifier A lambda function that can be used to modify the options before they go
//    *     into the AutoChooser
//    * @return SendableChooser populated with all autos
//    */
//   public  SendableChooser<Command> buildAutoChooserWithOptionsModifier(
//       String defaultAutoName,
//       Function<Stream<PathPlannerAuto>, Stream<PathPlannerAuto>> optionsModifier) {
//     if (!this.isConfigured()) {
//       throw new RuntimeException(
//           "this was not configured before attempting to build an auto chooser");
//     }

//     SendableChooser<Command> chooser = new SendableChooser<>();
//     List<String> autoNames = getAllAutoNames();

//     PathPlannerAuto defaultOption = null;
//     List<PathPlannerAuto> options = new ArrayList<>();

//     for (String autoName : autoNames) {
//       PathPlannerAuto auto = new PathPlannerAuto(autoName);

//       if (!defaultAutoName.isEmpty() && defaultAutoName.equals(autoName)) {
//         defaultOption = auto;
//       } else {
//         options.add(auto);
//       }
//     }

//     if (defaultOption == null) {
//       chooser.setDefaultOption("None", Commands.none());
//     } else {
//       chooser.setDefaultOption(defaultOption.getName(), defaultOption);
//       chooser.addOption("None", Commands.none());
//     }

//     optionsModifier
//         .apply(options.stream())
//         .forEach(auto -> chooser.addOption(auto.getName(), auto));

//     return chooser;
//   }

//   /**
//    * Get a list of all auto names in the project
//    *
//    * @return List of all auto names
//    */
//   public  List<String> getAllAutoNames() {
//     File[] autoFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/autos").listFiles();

//     if (autoFiles == null) {
//       return new ArrayList<>();
//     }

//     return Stream.of(autoFiles)
//         .filter(file -> !file.isDirectory())
//         .map(File::getName)
//         .filter(name -> name.endsWith(".auto"))
//         .map(name -> name.substring(0, name.lastIndexOf(".")))
//         .collect(Collectors.toList());
//   }

//   /**
//    * Get if this was configured for a holonomic drive train
//    *
//    * @return True if holonomic
//    */
//   public  boolean isHolonomic() {
//     if (!this.isConfigured()) {
//       throw new RuntimeException("this was not configured before use");
//     }

//     return isHolonomic;
//   }

//   /**
//    * Builds an auto command for the given auto name.
//    *
//    * @param autoName the name of the auto to build
//    * @return an auto command for the given auto name
//    */
//   public  Command buildAuto(String autoName) {
//     return new PathPlannerAuto(autoName);
//   }

//   /**
//    * Create a command to reset the robot's odometry to a given blue alliance pose
//    *
//    * @param bluePose The pose to reset to, relative to blue alliance origin
//    * @return Command to reset the robot's odometry
//    */
//   public  Command resetOdom(Pose2d bluePose) {
//     if (!this.isConfigured()) {
//       throw new RuntimeException("this was not configured before use");
//     }

//     return Commands.runOnce(
//         () -> {
//           boolean flip = shouldFlipPath.getAsBoolean();
//           if (flip) {
//             resetPose.accept(FlippingUtil.flipFieldPose(bluePose));
//           } else {
//             resetPose.accept(bluePose);
//           }
//         });
//   }

//   /** Functional interface for a function that takes 3 inputs */
//   @FunctionalInterface
//   public interface TriFunction<In1, In2, In3, Out> {
//     /**
//      * Apply the inputs to this function
//      *
//      * @param in1 Input 1
//      * @param in2 Input 2
//      * @param in3 Input 3
//      * @return Output
//      */
//     Out apply(In1 in1, In2 in2, In3 in3);
//   }
// }
