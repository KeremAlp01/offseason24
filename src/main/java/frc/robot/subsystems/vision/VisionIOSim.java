// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Add your docs here. */
public class VisionIOSim implements VisionIO {

  private final PhotonCamera camera = new PhotonCamera("camera");
  private final VisionSystemSim visionSim = new VisionSystemSim("camera");
  private final PhotonPoseEstimator odometry;
  private double pastTimestamp;

  public VisionIOSim() {
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    SimCameraProperties cameraProp = new SimCameraProperties();
    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);

    cameraProp.setCalibration(1280, 960, Rotation2d.fromDegrees(70.0));
    cameraProp.setCalibError(0, 0);

    cameraProp.setFPS(20);
    cameraProp.setAvgLatencyMs(30);
    cameraProp.setLatencyStdDevMs(4);

    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);

    Translation3d robotToCameraTrans = new Translation3d(0, 0, 1);
    Rotation3d robotToCameraRot = new Rotation3d(0, 0, 0);
    Transform3d robotToCamera = new Transform3d(robotToCameraTrans, robotToCameraRot);

    odometry = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, robotToCamera);
    odometry.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    visionSim.addAprilTags(tagLayout);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void setReferencePose(Pose2d pose) {
    odometry.setReferencePose(pose);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PhotonPipelineResult result = camera.getLatestResult();
    Optional<EstimatedRobotPose> currentPose =
        checkValidResult(result.targets) ? odometry.update(result) : Optional.empty();

    if (currentPose.isPresent()) {
      inputs.currentPose = currentPose.get().estimatedPose.toPose2d();
      inputs.timestamp = currentPose.get().timestampSeconds;
      inputs.x = currentPose.get().estimatedPose.getX();
      inputs.y = currentPose.get().estimatedPose.getY();
      inputs.rotation = currentPose.get().estimatedPose.getRotation().getAngle();
    } else {
      inputs.currentPose = null;
      inputs.timestamp = Double.NaN;
    }

    // inputs.allowEstimation = !currentPose.equals(Optional.empty());

    if (result.hasTargets() && currentPose.isPresent()) {
      inputs.hasTargets = true;
    } else {
      inputs.hasTargets = false;
    }

    inputs.isNew = false;
    if (pastTimestamp != inputs.timestamp) {
      inputs.isNew = true;
    }

    pastTimestamp = inputs.timestamp;
  }

  private boolean checkValidResult(List<PhotonTrackedTarget> result) {
    return true;
  }
}
