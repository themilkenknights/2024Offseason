package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double motorAppliedVolts = 0.0;
    public double motorTorqueCurrentAmps = 0;
    public double motorSupplyCurrentAmps = 0;

    public double encoderValue = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void setMotorVoltage(double volts) {}

  public default void setServo(double value) {}

  public default void resetEncoder(double position) {}
}
