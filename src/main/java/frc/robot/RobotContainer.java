// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.Constants.MotorConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private static final CommandPS5Controller jackController = new CommandPS5Controller(0);
  private static final CommandXboxController OperatorController = new CommandXboxController(1);

  public FuelSim fuelSim;
  // Dashboard
  private final LoggedDashboardChooser<Command> autoChooser;

  private boolean intakeDeployed = false;

  private double presetHoodPos = 0.0;
  private double presetRPM = 0.0;

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
    NamedCommands.registerCommand(
        "Intake Retract With Rollers", intake.RetractWithRollers(MaxShutter, IntakeMaxSpeed));

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
            () -> -jackController.getLeftY(),
            () -> -jackController.getLeftX(),
            () -> jackController.getRightX()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro Command- Will (Jack Has it for DEBUG)

    // ##########################################
    // AUTO FIRING SECTION
    // ##########################################
    jackController
        .R2()
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

    // Trigger usingPreset = OperatorController.rightTrigger();

    // Default (no operator toggle): distance-based shooting
    jackController
        .R2()
        .whileTrue(
            shooter
                .shooterDistanceToPosition(() -> drive.getDistanceToHub())
                .alongWith(
                    Commands.waitUntil(() -> shooter.isShooterAtSpeed())
                        .andThen(indexer.RunBothIndexer(1))));

    // Operator holding left bumper
    // jackController
    //     .R2()
    //     .and(usingPreset)
    //     .whileTrue(
    //         shooter.PresetShot(() -> presetHoodPos, () -> presetRPM)
    //             .alongWith(
    //                 Commands.waitUntil(() -> shooter.isShooterAtSpeed())
    //                     .andThen(indexer.RunBothIndexer(1))));

    jackController.R2().onFalse(shooter.hoodStop());

    // ##########################################
    // Intake Section
    // ##########################################
    jackController.L2().whileTrue(intake.IntakeCommand(IntakeCollect, IntakeMaxSpeed));
    jackController.L1().onTrue(intake.ReturnIntake());

    jackController.circle().whileTrue(indexer.RunStarWheels());
    // jackController.square().onTrue(shooter.ResetEncoder());

    // ##########################################
    // OPERATOR CONTROLLER - SHOT PRESETS
    // ##########################################

    OperatorController.a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  presetHoodPos = 0.0;
                  presetRPM = 2500;
                })); // Close
    OperatorController.b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  presetHoodPos = 0.12;
                  presetRPM = 3100;
                })); // Mid
    OperatorController.y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  presetHoodPos = 0.330;
                  presetRPM = 3250;
                })); // Far
    OperatorController.x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  presetHoodPos = 0.21;
                  presetRPM = 3900;
                })); // Very far

    OperatorController.rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                drive));

    OperatorController.leftBumper().onTrue(intake.SetIntakeCommand(MaxShutter));

    // ##########################################
    // OPERATOR MANUAL OVERRIDES
    // ##########################################

    OperatorController.povUp().whileTrue(intake.ManualIntakeAdjust(-0.07));
    OperatorController.povDown().whileTrue(intake.ManualIntakeAdjust(0.07));
    OperatorController.povLeft().whileTrue(shooter.ManualHoodAdjust(-0.012));
    OperatorController.povRight().whileTrue(shooter.ManualHoodAdjust(0.012));
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
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return false;
    if (DriverStation.isAutonomousEnabled()) return true;
    if (!DriverStation.isTeleopEnabled()) return false;

    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData.isEmpty()) return true; // assume active until data arrives

    if (gameData.charAt(0) != 'R' && gameData.charAt(0) != 'B') return true; // corrupt data
    boolean redInactiveFirst = gameData.charAt(0) == 'R';

    // Shift 1 is active for whichever alliance did NOT go inactive first
    boolean shift1Active = IsRed() ? !redInactiveFirst : redInactiveFirst;

    if (matchTime > Constants.AllianceShiftConstants.SHIFT_1_TIME) return true; // transition
    if (matchTime > Constants.AllianceShiftConstants.SHIFT_2_TIME) return shift1Active;
    if (matchTime > Constants.AllianceShiftConstants.SHIFT_3_TIME) return !shift1Active;
    if (matchTime > Constants.AllianceShiftConstants.END_GAME_TIME) return shift1Active;
    return true; // end game, always active
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
