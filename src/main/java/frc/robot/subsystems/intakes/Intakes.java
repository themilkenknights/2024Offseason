// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakes;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intakes extends SubsystemBase {
  public static enum intakeStates {
    IN,
    OUT,
    GROUND,
    GROUNDOUT,
    OFF
  }

  public Command setIntakesState(intakeStates state) {
    return new InstantCommand(
        () -> {
          io.setMotorStates(state);
        });
  }

  IntakesIO io;
  IntakesIOInputsAutoLogged inputs = new IntakesIOInputsAutoLogged();
  /** Creates a new Intakes. */
  public Intakes(IntakesIO io) {
    this.io = io;

    // if needed, switch Constants.currentMode

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intakes", inputs);
  }
}
