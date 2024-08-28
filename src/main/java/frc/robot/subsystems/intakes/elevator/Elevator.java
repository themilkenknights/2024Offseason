package frc.robot.subsystems.intakes.elevator;

import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  ElevatorIO io;

  public Elevator(ElevatorIO io) {
    this.io = io;

    // advantagescope
    stage.setColor(new Color8Bit(Color.kSilver));
    stage.append(new MechanismLigament2d("Intake", -0.31, 90));
  }

  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  @AutoLogOutput(key = "d")
  private Mechanism2d mech =
      new Mechanism2d(18 / 39.37, 30 / 39.37); // ,new Color8Bit(Color.kBlueViolet));

  private MechanismLigament2d stage =
      mech.getRoot("Elevator", 18 / 39.37, 0.1)
          .append(new MechanismLigament2d("Stage", 10 / 39.37, 90));

  private static final double kP = 1.3;
  private static final double kI = 0;
  private static final double kD = 0;
  TrapezoidProfile.Constraints profile = new TrapezoidProfile.Constraints(2000, 1400);
  ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, profile);
  private ElevatorFeedforward ffController = new ElevatorFeedforward(0, 0.43, 0.0283, 0.0007);

  public static enum Positions {
    AUTO(130),
    HP(140), // about 24.5 inches
    GROUND(0);

    private double value;

    private Positions(double value) {
      this.value = value;
    }
  };
  
  public InstantCommand setGoal(double goal) {
    
    return new InstantCommand(() -> pidController.setGoal(goal));
  }

  public InstantCommand setGoal(Positions goal) {
    return new InstantCommand(() -> pidController.setGoal(goal.value));
  }

  public Command goToPosition(Positions goal) {
    Command result = setGoal(goal);
    result.addRequirements(this);
    return result.andThen(waitUntil(pidController::atSetpoint));
  }

  public Command stow() {
    return setGoal(Positions.GROUND);
  }

  double lHeight;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    io.setMotorState(
        pidController.calculate(inputs.encoderValue)
            + ffController.calculate(pidController.getSetpoint().velocity));

    stage.setLength((inputs.encoderValue / Math.PI) / 40); // +0.3);
    SmartDashboard.putData(mech);
    Pose3d midStagePose = new Pose3d();
    boolean up = inputs.leftAppliedVolts >= 0.4;
    // pidController.getSetpoint().position <= pidController.getGoal().position|
    // pidController.atGoal();

    if (inputs.encoderValue > 60 & up) {
      if ((((inputs.encoderValue - 70) / Math.PI) / 80) >= lHeight) {
        midStagePose =
            new Pose3d(0, 0, (((inputs.encoderValue - 70) / Math.PI) / 80), new Rotation3d());
        lHeight = (((inputs.encoderValue - 70) / Math.PI) / 80);
      } else {
        midStagePose = new Pose3d(0, 0, lHeight, new Rotation3d());
      }
    } else if (inputs.encoderValue > 60) {
      midStagePose = new Pose3d(0, 0, lHeight, new Rotation3d());
    } else if (inputs.encoderValue < 60) {
      midStagePose = new Pose3d(0, 0, (((inputs.encoderValue) / Math.PI) / 80), new Rotation3d());
      lHeight = (((inputs.encoderValue + 80) / Math.PI) / 80);
    }
    if (up & !(inputs.encoderValue > 60)) {
      midStagePose = new Pose3d();
      lHeight = 0;
    }

    Logger.recordOutput(
        "Elevator",
        midStagePose,
        new Pose3d(0, 0, (inputs.encoderValue / Math.PI) / 80, new Rotation3d()),
        new Pose3d()); // new Pose3d(0, 0, (inputs.encoderValue / Math.PI) / 160, new Rotation3d())
  }

  public double getPositionError() {
    return pidController.getPositionError();
  }
}
