// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.Constants.MotorConstants.IndexerBeltMotorID;
import static frc.robot.Constants.MotorConstants.IndexerStarMotorID;
import static frc.robot.Constants.MotorConstants.IntakeCollect;
import static frc.robot.Constants.MotorConstants.IntakeDrivMotorID;
import static frc.robot.Constants.MotorConstants.IntakeMaxSpeed;
import static frc.robot.Constants.MotorConstants.IntakePosMotorID;
import static frc.robot.Constants.MotorConstants.IntakeStowed;
import static frc.robot.Constants.MotorConstants.MaxShutter;
import static frc.robot.Constants.MotorConstants.ShooterMotorID;
import static frc.robot.Constants.ShooterConstants.NO_VISION_FALLBACK_HOOD;
import static frc.robot.Constants.ShooterConstants.NO_VISION_FALLBACK_RPM;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CameraConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIO;
import frc.robot.subsystems.Indexer.IndexerIOSparkMax;
import frc.robot.subsystems.Indexer.IndexerIOStarSparkMax;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOSparkMax;
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
import frc.robot.util.ShiftUtil;
import frc.robot.util.ZoneShot;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Vision vision;
  private final Shooter shooter;

  private final LEDSubsystem ledSubsystem;
  // Controller
  private static final CommandPS5Controller jackController = new CommandPS5Controller(0);
  private static final CommandXboxController OperatorController = new CommandXboxController(1);

  public double slowSpeedMultiplier = 1;
  public FuelSim fuelSim;
  // Dashboard
  private final LoggedDashboardChooser<Command> autoChooser;

  // private boolean intakeDeployed = false;

  // private double presetHoodPos = 0.0;
  // private double presetRPM = 0.0;

  public RobotContainer() {
    fuelSim = new FuelSim();
    ledSubsystem = new LEDSubsystem();
    SmartDashboard.putNumber("Hood Angle", 0);
    SmartDashboard.putNumber("MotorVoltage", 0);
    SmartDashboard.putNumber("FEEDF", 0);

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
                new VisionIOReal("LeftCamera", CameraConstants.CAMERA_R_TRANSFORM_TO_ROBOT),
                new VisionIOReal("RightCamera", CameraConstants.CAMERA_L_TRANSFORM_TO_ROBOT));

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

    NamedCommands.registerCommand("Intake Deploy", intake.SetIntakeCommand(IntakeCollect));
    NamedCommands.registerCommand("Roller Enable", intake.RunIntakeShaft(IntakeMaxSpeed));
    NamedCommands.registerCommand("Intake Retract", intake.SetIntakeCommand(IntakeStowed));
    NamedCommands.registerCommand("Fire Preset Left Command", shooter.AutoDirectShot(0.2, -3000));
    NamedCommands.registerCommand(
        "Alignment",
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> 0,
            () -> 0,
            () -> {
              var robotPos = drive.getPose().getTranslation();
              var hubCenter = FieldConstants.getHubCenter(IsRed());
              return hubCenter.minus(robotPos).getAngle();
            }));
    NamedCommands.registerCommand("Indexer Start", indexer.RunBothIndexer(1));
    NamedCommands.registerCommand("Stop Shooter", shooter.MotorStop());
    NamedCommands.registerCommand("Stop Rollers", intake.StopIntakeShaft());
    NamedCommands.registerCommand("Stop Indexer", indexer.StopIndexer());
    NamedCommands.registerCommand("IntakeFeedPos", intake.SetIntakeCommand(MaxShutter));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Feild Relative Drive

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> (-jackController.getLeftY() * slowSpeedMultiplier),
            () -> (-jackController.getLeftX() * slowSpeedMultiplier),
            () -> (jackController.getRightX() * slowSpeedMultiplier)));

    // Intake slow Down Speed

    jackController.L2().onTrue(Commands.runOnce(() -> slowSpeedMultiplier = 0.5));
    jackController.L2().onFalse(Commands.runOnce(() -> slowSpeedMultiplier = 1.0));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro Command- Will (Jack Has it for DEBUG)

    // ##########################################
    // AUTO FIRING SECTION
    // ##########################################
    jackController
        .R2()
        .and(() -> !vision.isAllCamerasDisconnected())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -jackController.getLeftY(),
                () -> -jackController.getLeftX(),
                () -> ZoneShot.getTargetAngle(drive.getPose(), IsRed())));

    jackController
        .R2()
        .whileTrue(
            shooter
                .shooterDistanceOrFallback(
                    () -> drive.getDistanceToHub(),
                    () -> !vision.isAllCamerasDisconnected(),
                    NO_VISION_FALLBACK_HOOD,
                    NO_VISION_FALLBACK_RPM)
                .alongWith(
                    Commands.waitUntil(() -> shooter.isShooterAtSpeed())
                        .andThen(indexer.RunBothIndexer(1))));

    jackController.R2().onFalse(shooter.hoodStop());

    // ##########################################
    // Intake Section
    // ##########################################
    jackController.L2().whileTrue(intake.IntakeCommand(IntakeCollect, IntakeMaxSpeed));
    // jackController.L1().onTrue(intake.ReturnIntake());

    jackController.triangle().onTrue(intake.ZeroIntake());
    jackController.circle().whileTrue(indexer.RunStarWheels());

    // jackController.square().onTrue(shooter.ResetEncoder());

    // ##########################################
    // OPERATOR CONTROLLER - SHOT PRESETS
    // ##########################################

    OperatorController.start()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

    OperatorController.back()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

    OperatorController.a().onTrue(intake.ReturnIntake());
    OperatorController.b().onTrue(intake.ReturnIntake());
    OperatorController.x().onTrue(intake.ReturnIntake());
    OperatorController.y().onTrue(intake.ReturnIntake());
    OperatorController.rightBumper().onTrue(intake.ReturnIntake());
    OperatorController.rightTrigger().onTrue(intake.ReturnIntake());
    OperatorController.leftBumper().onTrue(intake.ReturnIntake());
    OperatorController.leftTrigger().onTrue(intake.ReturnIntake());

    // ##########################################
    // OPERATOR MANUAL OVERRIDES
    // ##########################################

    // OperatorController.povUp().whileTrue(intake.ManualIntakeAdjust(-0.07));
    // OperatorController.povDown().whileTrue(intake.ManualIntakeAdjust(0.07));
    // OperatorController.povLeft().whileTrue(shooter.ManualHoodAdjust(-0.012));
    // OperatorController.povRight().whileTrue(shooter.ManualHoodAdjust(0.012));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Drive getDrive() {
    return drive;
  }

  public static boolean IsRed() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public static Command createRumbleCommand(int controllerInt, double intensity, double duration) {
    if (controllerInt == 1) {
      return Commands.startEnd(
              () -> jackController.setRumble(GenericHID.RumbleType.kBothRumble, intensity),
              () -> jackController.setRumble(GenericHID.RumbleType.kBothRumble, 0))
          .withTimeout(duration);
    } else {
      return Commands.startEnd(
              () -> OperatorController.setRumble(GenericHID.RumbleType.kBothRumble, intensity),
              () -> OperatorController.setRumble(GenericHID.RumbleType.kBothRumble, 0))
          .withTimeout(duration);
    }
  }

  public static boolean HaveAllianceShift() {
    return ShiftUtil.getShiftInfo().active();
  }

  public static void triggerMissingDataRumble() {
    CommandScheduler.getInstance().schedule(createRumbleCommand(1, 1.0, 1.5));
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

  public boolean getL2Pressed() {
    return jackController.L2().getAsBoolean();
  }

  public boolean getR1Pressed() {
    return jackController.R1().getAsBoolean();
  }
}
