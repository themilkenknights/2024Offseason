package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;

public class LimelightIO_MT2 implements LimelightIO {

  @Override
  public void updateInputs(LimelightIOInputs inputs) {
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.LimelightName);

    inputs.robotPoseBlue = mt2.pose;
    inputs.lastTimestamp = mt2.timestampSeconds;
  }

  @Override
  public void setPose(Pose2d pose) {
    LimelightHelpers.SetRobotOrientation(
        Constants.LimelightName, pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
  }
}
