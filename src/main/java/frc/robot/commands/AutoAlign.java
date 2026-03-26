// package frc.robot.commands;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.math.ChassisSpeeds;
// import edu.wpi.first.math.MathUtil;
// import frc.robot.FieldConstants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.drive.Drive;
// import java.util.function.Supplier;
// import org.littletonrobotics.junction.Logger;

// public class AutoAlign {

//   private static final double ANGLE_KP = 5.0;
//   private static final double ANGLE_MAX_VELOCITY = 2 * Math.PI; // rad/s
//   private static final double ANGLE_MAX_ACCEL = 4 * Math.PI; // rad/s²
//   private static final double ANGLE_TOLERANCE = 0.05; // radians (~3 degrees)
//   private static final double ANGULAR_VELOCITY_TOLERANCE = 0.1; // rad/s

//   /**
//    * Command that aligns the robot to face a target angle and finishes when aligned
//    *
//    * @param drive Drive subsystem
//    * @param targetAngleSupplier Supplier that provides the target angle
//    * @return Command that finishes when aligned
//    */
//   public static Command alignToAngle(Drive drive, Supplier<Rotation2d> targetAngleSupplier) {

//     ProfiledPIDController angleController =
//         new ProfiledPIDController(
//             ANGLE_KP,
//             0.0,
//             0.0,
//             new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCEL));
//     angleController.enableContinuousInput(-Math.PI, Math.PI);
//     angleController.setTolerance(ANGLE_TOLERANCE, ANGULAR_VELOCITY_TOLERANCE);

//     return Commands.run(
//             () -> {
//               Rotation2d targetAngle = targetAngleSupplier.get();
//               Rotation2d currentAngle = drive.getRotation();

//               Logger.recordOutput("AutoAlign/TargetAngleDeg", targetAngle.getDegrees());
//               Logger.recordOutput("AutoAlign/CurrentAngleDeg", currentAngle.getDegrees());
//               Logger.recordOutput(
//                   "AutoAlign/ErrorDeg", targetAngle.minus(currentAngle).getDegrees());

//               // Calculate angular speed
//               double omega =
//                   MathUtil.applyDeadband(
//                       angleController.calculate(
//                           currentAngle.getRadians(), targetAngle.getRadians()),
//                       0.05);

//               Logger.recordOutput("AutoAlign/OmegaRadPerSec", omega);

//               // Apply rotation only (no linear movement)
//               ChassisSpeeds speeds = new ChassisSpeeds(0, 0, omega);

//               boolean isFlipped =
//                   DriverStation.getAlliance().isPresent()
//                       && DriverStation.getAlliance().get() == Alliance.Red;

//               drive.runVelocity(
//                   ChassisSpeeds.fromFieldRelativeSpeeds(
//                       speeds,
//                       isFlipped
//                           ? drive.getRotation().plus(new Rotation2d(Math.PI))
//                           : drive.getRotation()));
//             },
//             drive)
//         .beforeStarting(
//             () ->
//                 angleController.reset(
//                     drive.getRotation().getRadians(),
//                     drive.getFieldRelativeVelAngularVelRadPerSec()))
//         .until(() -> angleController.atSetpoint());
//   }

//   /**
//    * Command that aligns the robot to face the hub (speaker) and finishes when aligned
//    *
//    * @param drive Drive subsystem
//    * @return Command that finishes when aligned to hub
//    */
//   public static Command alignToHub(Drive drive) {
//     return alignToAngle(
//         drive,
//         () -> {
//           var robotPos = drive.getPose().getTranslation();
//           var hubCenter = FieldConstants.getHubCenter(RobotContainer.IsRed());
//           return hubCenter.minus(robotPos).getAngle();
//         });
//   }
// }
