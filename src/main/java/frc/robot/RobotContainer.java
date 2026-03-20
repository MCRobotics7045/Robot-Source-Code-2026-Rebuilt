// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.Constants.MotorConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CameraConstants;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.IndexerIOSparkMax;
import frc.robot.subsystems.Indexer.IndexerIOStarSparkMax;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOHoodMotor;
import frc.robot.subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOReal;
import frc.robot.subsystems.Vision.VisionIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.FuelSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Vision vision;
  private final Shooter shooter;

  // Controller
  private final CommandPS5Controller jackController = new CommandPS5Controller(0);
  private final CommandXboxController willController = new CommandXboxController(1);

  public FuelSim fuelSim;
  // Dashboard
  private final LoggedDashboardChooser<Command> autoChooser;

  private boolean intakeDeployed = false;

  private final AutoAlign autoAlign;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    fuelSim = new FuelSim();

    SmartDashboard.putNumber("Hood Angle", 0);
    SmartDashboard.putNumber("MotorVoltage", 0);

    switch (Constants.currentMode) {
      case REAL:
        // REAL Drive
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        intake = new Intake(new IntakeIOSparkMax(IntakePosMotorID, IntakeDrivMotorID)); // 37 38
        indexer =
            new Indexer(
                new IndexerIOSparkMax(IndexerBeltMotorID),
                new IndexerIOStarSparkMax(IndexerStarMotorID));
        shooter = new Shooter(new ShooterIOTalonFX(ShooterMotorID), new ShooterIOHoodMotor());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::setPose,
                new VisionIOReal("LeftCamera", CameraConstants.CAMERA_L_TRANSFORM_TO_ROBOT),
                new VisionIOReal("RightCamera", CameraConstants.CAMERA_R_TRANSFORM_TO_ROBOT));

        // // At Home
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {});
        // intake = new Intake(new IntakeIO() {});
        // indexer = new Indexer(new IndexerIO() {},new IndexerIO() {});
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        // shooter = new Shooter(new ShooterIO() {}, new ShooterIO() {});
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIO() {}, new IndexerIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::setPose,
                new VisionIOSim(drive::getPose, CameraConstants.CAMERA_L_TRANSFORM_TO_ROBOT),
                new VisionIOSim(drive::getPose, CameraConstants.CAMERA_R_TRANSFORM_TO_ROBOT));
        shooter = new Shooter(new ShooterIO() {}, new ShooterIO() {});
        configureFuelSim();
        fuelSim.enableAirResistance();
        fuelSim.start();
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {}, new IndexerIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement, drive::setPose, new VisionIO() {}, new VisionIO() {});

        shooter = new Shooter(new ShooterIO() {}, new ShooterIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoAlign = new AutoAlign(drive);
    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // // // Lock to 0° when A button is held
    // jackController
    //     .square()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -jackController.getLeftY(),
    //             () -> -jackController.getLeftX(),
    //             () -> Rotation2d.kZero));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro Command- Will (Jack Has it for DEBUG)
    jackController
        .circle()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    jackController
        .R1()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -jackController.getLeftY(),
                () -> -jackController.getLeftX(),
                () -> {
                  var robotPos = drive.getPose().getTranslation();
                  var hubCenter = FieldConstants.getHubCenter(IsRed());
                  return hubCenter.minus(robotPos).getAngle();
                }));

    // Command Section

    // If your reading this. Commands are split into two sections, 1 for Jack, 1 for Will. All
    // controlls will have comments for clarity

    // Jack Commands

    // Feild Relative Drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -jackController.getLeftY(),
            () -> -jackController.getLeftX(),
            () -> jackController.getRightX()));
    // jackController.R2().whileTrue(shooter.FireCommand(() -> drive.getDistanceToHub()));
    // Toggle Intake
    // Shooter Command

    // Will Commands

    // Elevator and Climber Control
    // Fine Tuning and Correction
    // Endgame Driving

    // controller.y().onTrue(uptake.runUptake());
    // controller.y().onTrue(intake.SetIntakeAngle(0));
    // jackController.triangle().onTrue(intake.SetIntakeAngle(0, -5.6));
    // jackController.L2().whileTrue(intake.RunIntakeShaft(0.5));
    // // controller.b().onTrue(shooter.Actuator());
    // jackController.R1().whileTrue(intake.ShutterBalls(-5.4, -2, 1));
    // jackController.L1().onTrue(shooter.Actuator());
    // jackController.R2().whileTrue(shooter.FireBlankCommand(-7.5));
    // jackController.cross().whileTrue(indexer.RunIndexerF(1.2));

    jackController.L2().whileTrue(intake.IntakeCommand(IntakeCollect, IntakeMaxSpeed));
    jackController.L1().onTrue(intake.ReturnIntake());
    jackController.square().onTrue(intake.ZeroIntake());

    jackController.R2().whileTrue(shooter.FireBlankCommand());
    jackController.triangle().whileTrue(indexer.RunBothIndexer(1));

    // do i need a stop button still?

    // Will Commands — Hood
    // While A is held, PID tracks the "Hood Angle" SmartDashboard value (0–100 → 0–1.2 enc rot)

    // B instantly stops the hood

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public static boolean IsRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  private void configureFuelSim() {
    fuelSim.spawnStartingFuel();
    fuelSim.registerRobot(
        0.889, // L to R
        0.8128, // F to B
        0.1524, // bumper height
        drive::getPose,
        drive::getFieldRelativeSpeeds);
    fuelSim.setSubticks(20);
  }
}
