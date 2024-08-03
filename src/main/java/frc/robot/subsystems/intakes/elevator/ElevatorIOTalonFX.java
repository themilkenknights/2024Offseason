package frc.robot.subsystems.intakes.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {
  TalonFX leftMotor = new TalonFX(Constants.IntakeElevator.Elevator.leftMotorCANID);
  TalonFX rightMotor = new TalonFX(Constants.IntakeElevator.Elevator.rightMotorCANID);

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.encoderValue =
        (leftMotor.getPosition().getValue() + rightMotor.getPosition().getValueAsDouble()) / 2;

    inputs.leftAppliedVolts = leftMotor.getMotorVoltage().getValue();
    inputs.rightAppliedVolts = rightMotor.getMotorVoltage().getValue();

    inputs.leftSupplyCurrentAmps = leftMotor.getSupplyCurrent().getValue();
    inputs.leftTorqueCurrentAmps = leftMotor.getTorqueCurrent().getValue();

    inputs.rightSupplyCurrentAmps = rightMotor.getSupplyCurrent().getValue();
    inputs.rightTorqueCurrentAmps = rightMotor.getTorqueCurrent().getValue();
  }

  public ElevatorIOTalonFX() {
    rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));
  }

  @Override
  public void setMotorState(double volts) {
    leftMotor.setVoltage(volts);
  }
}
