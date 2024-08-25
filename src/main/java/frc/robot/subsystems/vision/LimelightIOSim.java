package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Objects;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class LimelightIOSim implements LimelightIO {

  private static final AprilTagFieldLayout tagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
  Pose2d poseFromDrive = new Pose2d();
  // A vision system sim labelled as "main" in NetworkTables
  VisionSystemSim visionSim = new VisionSystemSim("main");

  // The PhotonCamera used in the real robot code.
  PhotonCamera camera = new PhotonCamera("cameraName");

  // The simulated camera properties
  SimCameraProperties cameraProp = new SimCameraProperties();

  // The simulation of this camera. Its values used in real robot code will be
  // updated.
  PhotonCameraSim cameraSim;

  // pose estimator
  PhotonPoseEstimator photonPoseEstimator;

  public LimelightIOSim() {

    visionSim.addAprilTags(tagFieldLayout);

    // A 640 x 480 camera with a 100 degree diagonal FOV.
    cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
    // Approximate detection noise with average and standard deviation error in
    // pixels.
    cameraProp.setCalibError(0.25, 0.08);
    // Set the camera image capture framerate (Note: this is limited by robot loop
    // rate).
    cameraProp.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);

    cameraSim = new PhotonCameraSim(camera, cameraProp);

    // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
    // pose,
    // (Robot pose is considered the center of rotation at the floor level, or Z =
    // 0)
    Translation3d robotToCameraTrl = new Translation3d(0.1, 0.1, 0.1);
    // and pitched 15 degrees up.
    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-30), 0);
    Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

    // Add this camera to the vision system simulation with the given
    // robot-to-camera transform.
    visionSim.addCamera(cameraSim, robotToCamera);
    photonPoseEstimator =
        new PhotonPoseEstimator(
            tagFieldLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera);
    photonPoseEstimator.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update(camera.getLatestResult());
  }

  @Override
  public void updateInputs(LimelightIOInputs inputs) {
    visionSim.update(poseFromDrive);

    Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose(poseFromDrive);
    if (estimatedPose.isPresent()) {
      EstimatedRobotPose estimatedRobotPose = estimatedPose.get();
      inputs.robotPoseBlue = estimatedRobotPose.estimatedPose.toPose2d();
      inputs.lastTimestamp = estimatedRobotPose.timestampSeconds;
      inputs.usedTags =
          estimatedRobotPose.targetsUsed.stream()
              .filter(Objects::nonNull)
              .map((n) -> tagFieldLayout.getTagPose(n.getFiducialId()).get())
              .toArray(Pose3d[]::new);

      inputs.n1 = 15;
      inputs.n2 = 15;
      inputs.n3 = 9999999;
    }
    // camera.getLatestResult().getMultiTagResult().estimatedPose.;
  }

  @Override
  public void setPose(Pose2d pose) {
    poseFromDrive = pose;
  }
}
