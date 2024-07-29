package frc.robot.subsystems.intakes.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};

    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};

    public double encoderValue = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * @param state the output to set the motor two
   */
  public default void setMotorState(double state) {}
  ;
}
