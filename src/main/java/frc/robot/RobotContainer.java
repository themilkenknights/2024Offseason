// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

/*
         ______________
        |[]            |
        |  __________  |
        |  | Radio  |  |
        |  | Shack  |  |
        |  |________|  |
        |   ________   |
        |   [ [ ]  ]   |
        \___[_[_]__]___|
    Remember to save often :)
 */

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Autos.AutoBuildTool;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveCommands.HPChoices;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intakes.Intakes;
import frc.robot.subsystems.intakes.Intakes.intakeStates;
import frc.robot.subsystems.intakes.IntakesIO;
import frc.robot.subsystems.intakes.IntakesIOSim;
import frc.robot.subsystems.intakes.IntakesIOTalonFX;
import frc.robot.subsystems.intakes.elevator.Elevator;
import frc.robot.subsystems.intakes.elevator.ElevatorIO;
import frc.robot.subsystems.intakes.elevator.ElevatorIOSim;
import frc.robot.subsystems.intakes.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.subsystems.vision.LimelightIO;
import frc.robot.subsystems.vision.LimelightIOSim;
import frc.robot.subsystems.vision.LimelightIO_MT2;
import frc.robot.util.NoteVisuals;
import frc.robot.util.NoteVisuals.VisualNote;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  AutoBuildTool autoBuildTool;
  // Subsystems
  private final Drive drive;
  private final Intakes intakes;
  private final Elevator elevator;
  private final Climb climb;
  private final LEDS leds;
  private final Limelight vision;
  // notes
  NoteVisuals noteVisuals = new NoteVisuals();
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  //Second Controller
  private final CommandXboxController controllerOP = new CommandXboxController(1);

  // Dashboard inputs

  private AutoBuildTool.dashboardSubsystem dashboardSubsystem;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    new VisualNote(new Pose3d(1, 00, 0, new Rotation3d()));
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        // new Drive(
        // new GyroIOPigeon2(false),
        // new ModuleIOSparkMax(0),
        // new ModuleIOSparkMax(1),
        // new ModuleIOSparkMax(2),
        // new ModuleIOSparkMax(3));
        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX() {});
        intakes = new Intakes(new IntakesIOTalonFX() {}, elevator);
        climb = new Climb(new ClimbIOTalonFX());
        vision = new Limelight(new LimelightIO_MT2(), drive);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        intakes = new Intakes(new IntakesIOSim(drive::getPose), elevator);
        climb = new Climb(new ClimbIOSim());
        vision = new Limelight(new LimelightIOSim(), drive);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        intakes = new Intakes(new IntakesIO() {}, elevator);
        climb = new Climb(new ClimbIO() {});
        vision = new Limelight(new LimelightIO() {}, drive);
        break;
    }
    /*
      ,'"`.
     /     \
    :       :
    :       :
     `.___,'  Easter egg
    */
    // create led subsystem after creating all the others

    leds = new LEDS(intakes::getBeambreak);
    SmartDashboard.putData(leds);

    RegisterPPCommands();
    // Set up auto routines

    // Set up SysId routines
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Forward)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Quasistatic Reverse)",
    // drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Forward)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Drive SysId (Dynamic Reverse)",
    // drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Flywheel SysId (Quasistatic Forward)",
    // flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Flywheel SysId (Quasistatic Reverse)",
    // flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    // "Flywheel SysId (Dynamic Forward)",
    // flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    // "Flywheel SysId (Dynamic Reverse)",
    // flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void RegisterPPCommands() {

    // Add them to named commands
    NamedCommands.registerCommand("AmpOut", intakes.AutoAmpOuttake());
    NamedCommands.registerCommand("IntakeGround", intakes.AutoGroundIntake());
    NamedCommands.registerCommand("GoDown", intakes.goDown());
    NamedCommands.registerCommand("GoUp", intakes.goUp());
    NamedCommands.registerCommand(
        "RUNROLLERS", intakes.setIntakesState(intakeStates.GROUND).asProxy().withTimeout(0.02));

    autoBuildTool = new AutoBuildTool();
    dashboardSubsystem = autoBuildTool.new dashboardSubsystem();
    dashboardSubsystem.getName();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Elevator
    // A on switch is where B on xbox is :( 
       // same with x and y
    controllerOP.y().onTrue(elevator.setGoal(140)); // Hewlett-Packard (HP)
    controllerOP.b().onTrue(elevator.setGoal(130)); // Amp
    controllerOP.a().onTrue(elevator.setGoal(0));   // Ground
    // drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    controller
        .povRight()
        .whileTrue(drive.run(() -> drive.runVelocity(new ChassisSpeeds(0.5, 0, 0))));
    controller
        .povLeft()
        .whileTrue(drive.run(() -> drive.runVelocity(new ChassisSpeeds(-0.5, 0, 0))));

    // climb
    controller.povUp().onTrue(climb.deploy());
    controller.povDown().onTrue(climb.climb());

    controller
        .leftBumper()
        .whileTrue(DriveCommands.autoDriveCommandTest(drive, new Pose2d(0, 0, new Rotation2d(0))));
    Command ampCommand =
        new SequentialCommandGroup(DriveCommands.driveToAmp(), intakes.AutoAmpOuttake());
    Command HPCommand =
        new SequentialCommandGroup(
            DriveCommands.driveToHP(), intakes.AutoHPin(), intakes.setDown());
    Command OTFcycleCommand = ampCommand.andThen(HPCommand).repeatedly();

    controller.a().whileTrue(OTFcycleCommand);

    controller
        .leftTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                    DriveCommands.driveToHP(HPChoices.HP_LEFT),
                    intakes.AutoHPin(),
                    intakes.setDown())
                .andThen(OTFcycleCommand.asProxy()));
    controller
        .rightTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                    DriveCommands.driveToHP(HPChoices.HP_RIGHT),
                    intakes.AutoHPin(),
                    intakes.setDown())
                .andThen(OTFcycleCommand.asProxy()));

    controller
        .rightTrigger()
        .and(controller.leftTrigger())
        .whileTrue(
            new SequentialCommandGroup(
                    DriveCommands.driveToHP(HPChoices.HP_MID),
                    intakes.AutoHPin(),
                    intakes.setDown())
                .andThen(OTFcycleCommand.asProxy()));
    // leds
    controller.rightBumper().whileTrue(leds.FlashOrange());
    controller.leftBumper().whileTrue(leds.FlashBlue());
    // controller.rightBumper().onTrue(intakes.goUp());
    // controller.leftBumper().onTrue(intakes.goDown());

    // controller.b().onTrue(intakes.AutoHPin());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // AutoBuildTool tool = new AutoBuildTool(intakes.AutoAmpOuttake(),
    // intakes.AutoGroundIntake());
    return autoBuildTool.getAutoCommand(); // .tool.getAutoCommand();
  }
}
