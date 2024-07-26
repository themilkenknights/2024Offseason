package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

/** To see notes on the field with AdvantageScope Loosely based on Mechanical Advantage's version */
public class NoteVisuals extends SubsystemBase {

  // instancing
  private static NoteVisuals instance;

  public static NoteVisuals getInstance() {
    if (instance == null) {
      instance = new NoteVisuals();
    }
    return instance;
  }

  private static ArrayList<VisualNote> visualNotes = new ArrayList<>();

  public static class VisualNote {
    public VisualNote(Pose3d pose) {
      this.pose = pose;
      this.id = visualNotes.size();
      visualNotes.add(this);
      timer.stop();
    }

    // public VisualNote(Pose3d pose, double ttl) {
    //   this.pose = pose;
    //   this.id = visualNotes.size();
    //   this.ttl = ttl;
    //   visualNotes.add(this);
    //   timer.start();
    // }

    boolean shouldDestroy() {
      return timer.hasElapsed(ttl);
    }

    Pose3d pose;
    int id;
    Timer timer = new Timer();
    double ttl = 15;

    public Pose3d getPose() {
      return pose;
    }

    public void setPose(Pose3d pose) {
      this.pose = pose;
    }

    @Override
    public void finalize() {
      visualNotes.remove(this);
    }
  }

  @Override
  public void periodic() {

    Pose3d[] poses =
        visualNotes.stream().filter(Objects::nonNull).map(n -> n.getPose()).toArray(Pose3d[]::new);
    visualNotes =
        new ArrayList<>(
            visualNotes.stream()
                .filter(Objects::nonNull)
                .filter(VisualNote::shouldDestroy)
                .toList());
    Logger.recordOutput("NoteVisualizer/Notes", poses);
  }

  public NoteVisuals(VisualNote... notes) {}
}
