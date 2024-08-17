package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimbIOSim implements ClimbIO {
  double volts;
  // private TalonFX Motor = new TalonFX(Climb.ClimbCANID);
  private ElevatorSim m_Sim =
      new ElevatorSim(
          DCMotor.getFalcon500(1).withReduction(15.34), 1, 10, 0.05, -0.01, 11.3, false, 0);

  public void updateInputs(ClimbIOInputs inputs) {

    if (volts > 0) {
      m_Sim.setInputVoltage(volts);
    }

    m_Sim.update(0.02);

    inputs.encoderValue = positionToRotations(m_Sim.getPositionMeters());
    inputs.motorTorqueCurrentAmps = m_Sim.getCurrentDrawAmps();

    inputs.motorAppliedVolts = volts;
  }

  private double positionToRotations(double positionMeters) {
    return positionMeters * 15.34 / Units.inchesToMeters(2);
  }

  @Override
  public void resetEncoder(double position) {
    m_Sim.setState(positionToRotations(position), 0);
  }

  @Override
  public void setMotorVoltage(double volts) {
    this.volts = volts;
    m_Sim.setInputVoltage(volts);
  }

  Timer servoTimer = new Timer();
  boolean servoActuated = false;
  boolean servoActuating = false;

  @Override
  public void setServo(double value) {
    servoActuating = true;

    if (value < 0.3) {
      servoActuated = false;
    } else {
      servoTimer.restart();
    }
  }
}
