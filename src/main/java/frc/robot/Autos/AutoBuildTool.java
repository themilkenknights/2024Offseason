package frc.robot.Autos;

// import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.Telem.field2d;

import com.choreo.lib.Choreo;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

public class AutoBuildTool {
  public class dashboardSubsystem extends SubsystemBase {
    Alert emptyAuto = new Alert("Auto", "Auto Is Empty!!!", AlertType.WARNING);
    Alert autoToLong = new Alert("Auto", "Auto may be greater then 15 seconds", AlertType.ERROR);
    double estimatedAutoLength = 0;

    @Override
    public void periodic() {

      // warnings
      if (autoClassChooser.get() == AutoClass.NONE
          & centerLineChooser.get() == CenterLineChoice.NONE) {
        emptyAuto.set(true);
      } else {
        emptyAuto.set(false);
      }
      estimatedAutoLength = preDelayInput.get() + preLeave.get();
      if (autoClassChooser.get() == AutoClass.DOUBLE) {
        estimatedAutoLength += 4;
      }
      if (autoClassChooser.get() == AutoClass.SINGLE) {
        estimatedAutoLength += 2;
      }
      if (centerLineChooser.get() != CenterLineChoice.NONE) {
        estimatedAutoLength += 3;
      }

      if (estimatedAutoLength >= 15) {
        autoToLong.set(true);
      } else {
        autoToLong.set(false);
      }

      // telem
      if (!RobotState.isEnabled()) {

        List<Pose2d> path = List.of();
        boolean flip =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;
        if (!flip) {

          if ((autoClassChooser.get() != AutoClass.NONE)) {
            path = List.of(Choreo.getTrajectory("FirstNote").getPoses());
            field2d.getObject("expectedTraj0").setPoses(path);
            if (autoClassChooser.get() == AutoClass.DOUBLE) {
              path = List.of(Choreo.getTrajectory("ToSecondNote").getPoses());
              field2d.getObject("expectedTraj1").setPoses(path);
              path = List.of(Choreo.getTrajectory("SecondNoteScore").getPoses());
              field2d.getObject("expectedTraj2").setPoses(path);
            } else {
              clearFieldTraj(1);
              clearFieldTraj(2);
            }
          } else {
            clearFieldTraj(0);
            clearFieldTraj(1);
            clearFieldTraj(2);
          }
          if (centerLineChooser.get() != CenterLineChoice.NONE) {
            // path.addAll(List.of(Choreo.getTrajectory(centerLineChooser.get().toString()).getPoses()));
            path = List.of(Choreo.getTrajectory(centerLineChooser.get().toString()).getPoses());
            field2d.getObject("expectedTraj3").setPoses(path);
          } else {
            field2d.getObject("expectedTraj3").setTrajectory(new Trajectory());
          }
        } else {
          if ((autoClassChooser.get() != AutoClass.NONE)) {
            path = List.of(Choreo.getTrajectory("FirstNote").flipped().getPoses());
            field2d.getObject("expectedTraj0").setPoses(path);
            if (autoClassChooser.get() == AutoClass.DOUBLE) {
              path = List.of(Choreo.getTrajectory("ToSecondNote").flipped().getPoses());
              field2d.getObject("expectedTraj1").setPoses(path);
              path = List.of(Choreo.getTrajectory("SecondNoteScore").flipped().getPoses());
              field2d.getObject("expectedTraj2").setPoses(path);
            } else {
              clearFieldTraj(1);
              clearFieldTraj(2);
            }
          } else {
            clearFieldTraj(0);
            clearFieldTraj(1);
            clearFieldTraj(2);
          }
          if (centerLineChooser.get() != CenterLineChoice.NONE) {
            // path.addAll(List.of(Choreo.getTrajectory(centerLineChooser.get().toString()).getPoses()));
            path =
                List.of(
                    Choreo.getTrajectory(centerLineChooser.get().toString()).flipped().getPoses());
            field2d.getObject("expectedTraj3").setPoses(path);
          } else {
            field2d.getObject("expectedTraj3").setTrajectory(new Trajectory());
          }
        }
      } else {

        clearFieldTraj();
      }
    }

    private void clearFieldTraj() {
      field2d.getObject("expectedTraj0").setTrajectory(new Trajectory());
      field2d.getObject("expectedTraj1").setTrajectory(new Trajectory());
      field2d.getObject("expectedTraj2").setTrajectory(new Trajectory());
      field2d.getObject("expectedTraj3").setTrajectory(new Trajectory());
    }

    private void clearFieldTraj(int index) {
      field2d.getObject("expectedTraj" + index).setTrajectory(new Trajectory());
    }
  }

  LoggedDashboardChooser<AutoClass> autoClassChooser =
      new LoggedDashboardChooser<>("Auto Class Choice");

  LoggedDashboardNumber preDelayInput = new LoggedDashboardNumber("pre auto delay", 0);

  LoggedDashboardChooser<CenterLineChoice> centerLineChooser =
      new LoggedDashboardChooser<>("Center Line Choice");

  LoggedDashboardNumber preLeave = new LoggedDashboardNumber("delay after notes Seconds", 0);
  Alert AutoReminder = new Alert("Remember to select an Auto", AlertType.INFO);

  public AutoBuildTool() {
    AutoReminder.set(true);
    autoClassChooser.addDefaultOption("Single", AutoClass.SINGLE);
    autoClassChooser.addOption("Double", AutoClass.DOUBLE);
    autoClassChooser.addOption("None", AutoClass.NONE);

    centerLineChooser.addDefaultOption("Centerline note", CenterLineChoice.CENTERLINE_NOTE);
    centerLineChooser.addOption("centerline", CenterLineChoice.CENTERLINE);
    centerLineChooser.addOption("leave wing", CenterLineChoice.CENTERLINE_POST_WING);
    centerLineChooser.addOption("stay in wing", CenterLineChoice.WING);
    centerLineChooser.addOption("stay at amp", CenterLineChoice.NONE);
    // TODO: add backup option

    AutoClassCommands.put(AutoClass.SINGLE, AutoBuilder.buildAuto("SINGLE"));
    AutoClassCommands.put(AutoClass.DOUBLE, AutoBuilder.buildAuto("DOUBLE"));
    AutoClassCommands.put(AutoClass.NONE, Commands.none());
  }

  public Command getAutoCommand() {
    Command autoCommand;

    autoCommand =
        new SequentialCommandGroup(
            waitSeconds(preDelayInput.get()),
            CommandUtil.wrappedEventCommand(AutoClassCommands.get(autoClassChooser.get())),
            waitSeconds(preLeave.get()));

    if (centerLineChooser.get() != CenterLineChoice.NONE) {
      autoCommand =
          autoCommand.andThen(
              AutoBuilder.followPath(
                  PathPlannerPath.fromChoreoTrajectory(centerLineChooser.get().toString())));
    }

    return autoCommand;
  }

  HashMap<AutoClass, Command> AutoClassCommands = new HashMap<>();

  static enum AutoClass {
    /** Place a preloaded note */
    SINGLE,
    /** Place a preloaded and the note closet to the Amp */
    DOUBLE,
    /** Do nothing the whole time */
    NONE
  }

  static enum CenterLineChoice {
    /** Drive to and Pick up the not on the centerline closest to the amp. */
    CENTERLINE_NOTE,
    /** Go towards the centerline, but not quite to the the note */
    CENTERLINE,
    /** Exit the wing and stops */
    CENTERLINE_POST_WING,
    /** Stays in the wing (but leaves the starting zone) */
    WING,
    NONE
  }
}
