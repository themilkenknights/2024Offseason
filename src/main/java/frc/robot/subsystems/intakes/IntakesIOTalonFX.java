package frc.robot.subsystems.intakes;

import static frc.robot.Constants.IntakeElevator.Intakes;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.intakes.Intakes.intakeStates;

public class IntakesIOTalonFX implements IntakesIO {
  TalonFX topMotor = new TalonFX(Intakes.topIntakeCANID);
  TalonFX midMotor = new TalonFX(Intakes.midIntakeCANID);
  TalonFX bottomMotor = new TalonFX(Intakes.bottomIntakeCANID);

  /** MotorStates */
  private class IntakeMotorStates {
    double top, mid, bottom;

    public IntakeMotorStates(double top, double mid, double bottom) {
      this.top = top;
      this.mid = mid;
      this.bottom = bottom;
    }

    public void apply() {
      topMotor.setVoltage(top);
      midMotor.setVoltage(mid);
      bottomMotor.setVoltage(bottom);
    }
  }

  private final IntakeMotorStates IN = new IntakeMotorStates(12, 12, 0);
  private final IntakeMotorStates OUT = new IntakeMotorStates(-12, -12, 0);
  private final IntakeMotorStates GROUND = new IntakeMotorStates(0, 0, 0);
  private final IntakeMotorStates GROUNDOUT = new IntakeMotorStates(12, -12, 0);
  private final IntakeMotorStates OFF = new IntakeMotorStates(12, -12, 0);

  {
  }

  @Override
  public void setBottomMotorStates(intakeStates state) {
    throw new UnsupportedOperationException("not implemented");
  }

  @Override
  public void setMotorStates(intakeStates state) {
    switch (state) {
      case IN:

      case OUT, GROUND, GROUNDOUT, OFF:
        break;

      default:
        break;
    }
  }

  @Override
  public void setTopMotorStates(intakeStates state) {
    throw new UnsupportedOperationException("not implemented");
  }

  @Override
  public void updateInputs(IntakesIOInputs inputs) {
    // TODO Auto-generated method stub
    IntakesIO.super.updateInputs(inputs);
  }
 }
