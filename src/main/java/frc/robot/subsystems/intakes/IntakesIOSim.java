package frc.robot.subsystems.intakes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.intakes.Intakes.intakeStates;
import frc.robot.util.NoteVisuals.VisualNote;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class IntakesIOSim implements IntakesIO {
  private enum noteState {
    INTAKING,
    OUTTAKING,
    HAS,
    NONE
  }

  intakeStates lastState = intakeStates.OFF;

  noteState m_State = noteState.HAS;
  boolean estimatedBeambreak = true;
  Timer timeOfIntaking = new Timer();
  double ttNote = 0;
  Supplier<Pose2d> poseSupplier;

  private static Transform3d transform3d = new Transform3d(0.1, 0, 0.4, new Rotation3d(0, -45, 0));

  public IntakesIOSim(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  VisualNote note;

  @Override
  public void updateInputs(IntakesIOInputs inputs) {
    Logger.recordOutput("st", m_State);
    Logger.recordOutput("timer", timeOfIntaking.get());
    if (timeOfIntaking.hasElapsed(ttNote) && m_State == noteState.INTAKING) {
      estimatedBeambreak = false;
      m_State = noteState.HAS;
    } else if (timeOfIntaking.hasElapsed(ttNote) && m_State == noteState.OUTTAKING) {
      estimatedBeambreak = true;
      m_State = noteState.NONE;
    }
    inputs.beambreak = estimatedBeambreak;
    if (!estimatedBeambreak) {
      if (note == null) {
        note = new VisualNote(new Pose3d(poseSupplier.get()).transformBy(transform3d));
      } else {
        note.setPose(new Pose3d(poseSupplier.get()).transformBy(transform3d));
      }

      // note.setPose();
    } else if (m_State == noteState.NONE && note != null) {
      note.finalize();
      note = null;
    }
  }

  @Override
  public void setMotorStates(intakeStates state) {
    if (state != lastState) {
      switch (state) {
        case GROUND:
          ttNote = 1;
          timeOfIntaking.restart();
          m_State = noteState.INTAKING;
          estimatedBeambreak = true;
          break;
        case GROUNDOUT:
          ttNote = 1;
          timeOfIntaking.restart();
          m_State = noteState.OUTTAKING;
          break;
        case IN:
          ttNote = 0.75;
          timeOfIntaking.restart();
          m_State = noteState.INTAKING;
          estimatedBeambreak = true;
          break;
        case OUT:
          ttNote = 0.75;
          timeOfIntaking.restart();
          m_State = noteState.OUTTAKING;
          break;

        default:
          break;
      }
    }
    lastState = state;
    Logger.recordOutput("intake state", state);
  }
}
