package frc.robot.Autos;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutoBuildTool {

  public static class Triggers {
    // public Trigger SetRollersIn(){
    // return new Trigger(null);
    // }

    public Trigger SetRollersOut() {
      return new Trigger(() -> rollersOut);
    }

    public Trigger SetRollersGround() {
      return new Trigger(() -> rollersIn);
    }

    public Trigger SetElevatorHP() {
      return new Trigger(() -> elevatorUp);
    }

    public Trigger SetElevatorDown() {
      return new Trigger(() -> !elevatorUp);
    }

    public Command GoDown(){
        return NamedCommands.getCommand("GoDown");
    }
  }

  private static boolean rollersIn = false;
  private static boolean rollersOut = false;
  private static boolean elevatorUp = false;

  public static class AutonomousSequence {

    public AutonomousSequence(
        AutoClass autoClass,
        CenterLineChoice centerLineChoice,
        double preDelay,
        double preCenterLineDelay) {
      this.autoClass = autoClass;
      this.centerLineChoice = centerLineChoice;
      this.preDelay = preDelay;
      this.preCenterLineDelay = preCenterLineDelay;
    }

    AutoClass autoClass;
    CenterLineChoice centerLineChoice;
    double preDelay;
    double preCenterLineDelay;
    private Command autoCommand;

    private void build() {
      PathPlannerPath mainPath = PathPlannerPath.fromChoreoTrajectory("FirstNote");
      Command mainPathCommand = AutoBuilder.followPath(mainPath);
      PathPlannerPath secondPath;

      // create a SequentialCommandGroup with the delay and the main path;
      if (preDelay > 0) {
        autoCommand = new SequentialCommandGroup(GoDown(),waitSeconds(preDelay));
      } else {
        autoCommand = new SequentialCommandGroup(); // (GoDown());
      }

      // add main path
      if (autoClass != AutoClass.NONE) {
        autoCommand = new SequentialCommandGroup(mainPathCommand);
      }

      // add the post delay
      if (preCenterLineDelay > 0) {
        autoCommand = autoCommand.andThen(waitSeconds(preCenterLineDelay));
      }
      // add the center line path
      if (centerLineChoice != CenterLineChoice.NONE) {
        secondPath = PathPlannerPath.fromChoreoTrajectory(centerLineChoice.toString());
        autoCommand = autoCommand.andThen(AutoBuilder.followPath(secondPath));
      }
    }

    public Command getCommand() {
      if (autoCommand == null) {
        build();
      }
      return autoCommand;
    }
    ;
  }

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
