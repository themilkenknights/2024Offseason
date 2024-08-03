package frc.robot.subsystems.intakes.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ElevatorIOSim implements ElevatorIO {
  private ElevatorSim elevatorSim =
      new ElevatorSim(DCMotor.getFalcon500(2), 25, 10, 0.0127, 0, 0.9144, true, 0);
  double lv = 0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.update(0.02);
    inputs.encoderValue = elevatorSim.getPositionMeters() * 40 * 5.714;
    inputs.leftAppliedVolts = lv;
    inputs.rightAppliedVolts = lv;

    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

    inputs.leftTorqueCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.leftSupplyCurrentAmps = RoboRioSim.getVInCurrent();

    inputs.rightTorqueCurrentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.rightSupplyCurrentAmps = RoboRioSim.getVInCurrent();
  }

  @Override
  public void setMotorState(double volts) {
    lv = volts;
    elevatorSim.setInputVoltage(volts);
  }
}
