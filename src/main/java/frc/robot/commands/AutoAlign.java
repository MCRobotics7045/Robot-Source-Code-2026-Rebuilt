// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.FieldConstants;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.drive.Drive;

// public class AutoAlign extends Command {

//   private final Drive drive;
//   private final PIDController rotationController;
//   private double rotationSpeed;

//   public AutoAlign(Drive drive) {
//     this.drive = drive;
//     this.rotationController = new PIDController(0.05, 0.0, 0.05);
//     rotationController.enableContinuousInput(-Math.PI, Math.PI);
//     rotationController.setTolerance(0.05);
//   }

//   @Override
//   public void initialize() {
//     System.out.println("AutoAlign started...");
//   }

//   @Override
//   public void execute() {
//     Translation2d robotPos = drive.getPose().getTranslation();
//     Translation2d hubCenter = FieldConstants.getHubCenter(RobotContainer.IsRed());
//     Rotation2d targetAngle = hubCenter.minus(robotPos).getAngle();
//     Rotation2d currentAngle = drive.getRotation();

//     rotationSpeed =
//         rotationController.calculate(currentAngle.getRadians(), targetAngle.getRadians());

//     System.out.println(
//         "Target: "
//             + targetAngle.getDegrees()
//             + " Current: "
//             + currentAngle.getDegrees()
//             + " Speed: "
//             + rotationSpeed);
//   }

//   public Rotation2d getRotationSpeed() {
//     return new Rotation2d(rotationSpeed);
//   }

//   @Override
//   public boolean isFinished() {
//     return rotationController.atSetpoint();
//   }

//   @Override
//   public void end(boolean interrupted) {
//     System.out.println("AutoAlign ended...");
//   }
// }
// NOT USED AT ALL>>>> JUST A REFRENCE
