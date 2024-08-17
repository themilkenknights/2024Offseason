package frc.robot.subsystems.climb;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climb.gains;
import frc.robot.Constants.Climb.setpoints;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {

  public enum Positions {
    UP(setpoints.upSetpoint),
    DOWN(setpoints.downSetpoint);

    public final double value;

    private Positions(double value) {
      this.value = value;
    }
  }

  PIDController m_controller = new PIDController(gains.GainUpP, gains.gainI, gains.gainD);

  ClimbIO io;
  ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  boolean outputEnabled = true;

  /** Creates a new ExampleSubsystem. */
  public Climb(ClimbIO io) {
    this.io = io;
    m_controller.setTolerance(gains.tolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);

    io.setMotorVoltage(m_controller.calculate(inputs.encoderValue) / 5);
  }

  // public Command goUp(){
  // return runOnce(()->{
  // m_controller.setP(gains.GainUpP);
  // m_controller.setSetpoint(setpoints.upSetpoint);
  // });
  // }
  private void disableOutput() {
    outputEnabled = false;
  }

  private void enableOutput() {
    outputEnabled = true;
  }

  public Command goToHeight(double rotations) {
    return new SequentialCommandGroup(
        unlock(),
        runOnce(
            () -> {
              // if the setpoint is higher, use the up gains
              if (rotations >= m_controller.getSetpoint()) {
                m_controller.setP(gains.GainUpP);
              } else {
                // otherwise use the climb gains
                m_controller.setP(gains.GainDownP);
              }
              m_controller.setSetpoint(rotations);
            }),
        waitUntil(() -> m_controller.atSetpoint()));
  }

  public Command goToHeight(Positions height) {
    return goToHeight(height.value);
  }

  public Command lock() {
    return new SequentialCommandGroup(
        runOnce(
            () -> {
              disableOutput();
              io.setServo(setpoints.servoLockedSetpoint);
            }),
        waitSeconds(setpoints.servoDelay));
  }

  public Command unlock() {
    return new SequentialCommandGroup(
        runOnce(() -> io.setServo(setpoints.servoOpenSetpoint)),
        waitSeconds(setpoints.servoDelay),
        runOnce(() -> enableOutput()));
  }

  public Command climb() {
    return new SequentialCommandGroup(goToHeight(Positions.DOWN), lock());
  }

  public Command deploy() {
    return new SequentialCommandGroup(goToHeight(Positions.UP));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
