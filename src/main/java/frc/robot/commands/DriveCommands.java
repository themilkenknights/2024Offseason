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

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.subsystems.drive.AutoAlignController;
import frc.robot.subsystems.drive.Drive;
import java.util.Map;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {
  }

  public static Command autoDriveCommandTest(Drive drive, Pose2d target) {
    AutoAlignController controller = new AutoAlignController(target);
    return Commands.run(
        () -> {
          drive.runVelocity(controller.calculate(drive.getPose()));
        },
        drive);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude = MathUtil.applyDeadband(
              Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calculate new linear velocity
          Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
              .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
              .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  public static Command driveToAmp() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Amp aligner");
    // if (Constants.isFlipped.getAsBoolean()) {
    // path.flipPath();
    // }
    AutoBuilder.buildAuto("DOUBLE");
    return AutoBuilder.pathfindThenFollowPath(path, new PathConstraints(3.7, 3.5, 360, 540));
  }

  public static enum HPChoices {
    HP_LEFT,
    HP_RIGHT,
    HP_MID
  }

  private static HPChoices lastChoice = HPChoices.HP_RIGHT;

  private static class setLastHP extends Command {
    private HPChoices hp;

    setLastHP(HPChoices choice) {
      hp = choice;
    }

    @Override
    public boolean isFinished() {
      return true;
    }

    @Override
    public void execute() {
      lastChoice = hp;
    }
  }

  public static Command driveToHP(HPChoices hp) {
    lastChoice = hp;
    // PathPlannerPath path = PathPlannerPath.fromPathFile(hp.toString());
    return AutoBuilder.pathfindThenFollowPath(
        PathPlannerPath.fromPathFile(hp.toString()), new PathConstraints(3.7, 3.5, 360, 540))
        .alongWith(new setLastHP(hp));
  }

  private static Command HP_LEFT = AutoBuilder.pathfindThenFollowPath(
      PathPlannerPath.fromPathFile(HPChoices.HP_LEFT.toString()),
      new PathConstraints(3.7, 3.5, 360, 540));

  private static Command HP_MID = AutoBuilder.pathfindThenFollowPath(
      PathPlannerPath.fromPathFile(HPChoices.HP_MID.toString()),
      new PathConstraints(3.7, 3.5, 360, 540));

  private static Command HP_RIGHT = AutoBuilder.pathfindThenFollowPath(
      PathPlannerPath.fromPathFile(HPChoices.HP_RIGHT.toString()),
      new PathConstraints(3.7, 3.5, 360, 540));

  public static Command driveToHP() {

    // PathPlannerPath path = PathPlannerPath.fromPathFile(hp.toString());
    // select the last path
    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(HPChoices.HP_LEFT, HP_LEFT),
            Map.entry(HPChoices.HP_RIGHT, HP_RIGHT),
            Map.entry(HPChoices.HP_MID, HP_MID)),
        () -> lastChoice);
  }
}
