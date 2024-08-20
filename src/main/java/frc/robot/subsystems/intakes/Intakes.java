// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakes;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intakes.elevator.Elevator;
import frc.robot.subsystems.intakes.elevator.Elevator.Positions;
import org.littletonrobotics.junction.Logger;

public class Intakes extends SubsystemBase {
  private Elevator intakeElevator;

  public Intakes(IntakesIO io, Elevator elevator) {
    this.io = io;
    this.intakeElevator = elevator;
    // if needed, switch Constants.currentMode

    // defaulting
    // final Command defaultCommandGroup = setIntakesState(intakeStates.OFF);
    // defaultCommandGroup.addRequirements(this);
    // setDefaultCommand(defaultCommandGroup);
  }

  // private final double elevatorspeed = .6;
  // private final double groundspeed = 0.6;
  private final double waittime = 0.05;
  // private final double waittimeGround = 0.24;

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intakes", inputs);
  }

  public Command AutoHPin() {
    return new SequentialCommandGroup(
        intakeElevator
            .goToPosition(Elevator.Positions.HP)
            .until(() -> intakeElevator.getPositionError() < 10),
        setIntakesState(intakeStates.IN),
        TopIntakeByBeambreak(),
        runOnce(
            () -> {
              intakeElevator.setGoal(Positions.GROUND);
            }));
  }

  public Command TopIntakeByBeambreak() {
    return new SequentialCommandGroup(
        setIntakesState(intakeStates.IN),
        waitUntil(() -> !inputs.beambreak),
        waitSeconds(waittime),
        setIntakesState(intakeStates.OFF));
  }

  public Command TopOuttakeByBeambreak() {
    return new SequentialCommandGroup(
        setIntakesState(intakeStates.OUT),
        waitUntil(() -> inputs.beambreak),
        waitSeconds(waittime),
        setIntakesState(intakeStates.OFF));
  }

  public Command AmpOuttake() {
    return new SequentialCommandGroup(
            setIntakesState(intakeStates.OUT), TopOuttakeByBeambreak(), waitSeconds(0.1))
        .withName("Amp Outtake");
  }

  public Command AutoAmpOuttake() {
    return new SequentialCommandGroup(
            setIntakesState(intakeStates.OFF),
            intakeElevator
                .goToPosition(Elevator.Positions.AUTO)
                .until(() -> intakeElevator.getPositionError() < 10),
            TopOuttakeByBeambreak(),
            intakeElevator.setGoal(Elevator.Positions.GROUND))
        .withName("Amp Outtake");
  }

  public Command goUp() {
    return new ParallelCommandGroup(
        setIntakesState(intakeStates.OFF), intakeElevator.goToPosition(Elevator.Positions.HP));
  }

  public Command goDown() {
    return new ParallelCommandGroup(
        setIntakesState(intakeStates.OFF), intakeElevator.goToPosition(Elevator.Positions.GROUND));
  }

  public Command setDown() {
    return intakeElevator.setGoal(Positions.GROUND);
  }

  public Command HPin() {
    return new SequentialCommandGroup(setIntakesState(intakeStates.IN), TopIntakeByBeambreak());
  }

  public Command AutoGroundIntake() {
    return new SequentialCommandGroup(
        intakeElevator
            .goToPosition(Elevator.Positions.GROUND)
            .until(() -> intakeElevator.getPositionError() < 10),
        setIntakesState(intakeStates.GROUND));
  }
}
