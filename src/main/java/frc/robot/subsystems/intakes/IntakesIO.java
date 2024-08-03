package frc.robot.subsystems.intakes;

import org.littletonrobotics.junction.AutoLog;

public interface IntakesIO {
  @AutoLog
  public static class IntakesIOInputs {
    public boolean beambreak;

    public double topAppliedVolts = 0.0;
    public double topTorqueCurrentAmps = 0;
    public double topSupplyCurrentAmps = 0;

    public double midAppliedVolts = 0.0;
    public double midTorqueCurrentAmps = 0;
    public double midSupplyCurrentAmps = 0;

    public double groundAppliedVolts = 0.0;
    public double groundTorqueCurrentAmps = 0;
    public double groundSupplyCurrentAmps = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakesIOInputs inputs) {}

  /**
   * @param state the intakeState of the intakes sets all the motors for the rollers
   */
  public default void setMotorStates(Intakes.intakeStates state) {}

  /**
   * @param state the intakeState of the intakes sets the HP motors for the rollers
   */
  public default void setTopMotorStates(Intakes.intakeStates state) {}

  /**
   * @param state the intakeState of the intakes sets the ground motors for the rollers
   */
  public default void setBottomMotorStates(Intakes.intakeStates state) {}
}
