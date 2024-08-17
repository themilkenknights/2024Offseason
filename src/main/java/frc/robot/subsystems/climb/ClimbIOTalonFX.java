package frc.robot.subsystems.climb;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;

public class ClimbIOTalonFX implements ClimbIO {
  TalonFX Motor = new TalonFX(Constants.Climb.ClimbCANID);
  Servo Servo = new Servo(Constants.Climb.ClimbServoPORT);

  @Override
  public void setServo(double value) {
    Servo.set(value);
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    inputs.motorAppliedVolts = Motor.getMotorVoltage().getValue();
    inputs.motorSupplyCurrentAmps = Motor.getSupplyCurrent().getValue();
    inputs.motorTorqueCurrentAmps = Motor.getTorqueCurrent().getValue();

    inputs.encoderValue = Motor.getPosition().getValueAsDouble();
  }

  @Override
  public void setMotorVoltage(double volts) {
    Motor.setVoltage(volts);
  }

  @Override
  public void resetEncoder(double position) {
    Motor.setPosition(position);
  }
}
