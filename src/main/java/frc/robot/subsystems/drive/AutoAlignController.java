package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class AutoAlignController {
  PIDController xController = new PIDController(0.001, 0, 1);
  PIDController yController = new PIDController(0.001, 0, 1);
  PIDController theta = new PIDController(0.001, 0, 1);

  Pose2d target;

  public AutoAlignController(Pose2d target) {
    this.target = target;
    theta.enableContinuousInput(-180, 180);
  }

  public ChassisSpeeds calculate(Pose2d measurement) {

    double xSpeed = xController.calculate(measurement.getX(), target.getX()); // , 3, 3);
    double ySpeed = yController.calculate(measurement.getY(), target.getY()); // , 3, 3);
    double omega =
        -Units.degreesToRadians(
            theta.calculate(
                measurement.getRotation().getDegrees(), target.getRotation().getDegrees()));

    return new ChassisSpeeds(xSpeed, ySpeed, 0);
  }

  void setSetpoint(Pose2d setPoint) {
    this.target = setPoint;
  }
}
