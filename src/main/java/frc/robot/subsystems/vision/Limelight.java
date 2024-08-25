package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class Limelight extends SubsystemBase {
  LimelightIO io;
  LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();

  Drive drive;

  public Limelight(LimelightIO io, Drive drive) {
    this.io = io;
    this.drive = drive;
  }

  @Override
  public void periodic() {

    io.setPose(drive.getPose());
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    drive.acceptVisionMeasurement(
        inputs.robotPoseBlue,
        inputs.lastTimestamp,
        VecBuilder.fill(inputs.n1, inputs.n2, inputs.n3));
  }
}
