// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs;
  private final PoseAndTimestamp results = new PoseAndTimestamp(new Pose2d(), 0);

  /** Creates a new V. */
  public Vision(VisionIO io) {
    this.io = io;
    inputs = new VisionIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    Logger.processInputs("Vision", inputs);
    io.updateInputs(inputs);

    // if(inputs.allowEstimation){
    // drive.addVisionMeasurement(inputs.estimatedPose, inputs.timestamp);
    // }
    // This method will be called once per scheduler run
    if (inputs.hasTargets && inputs.isNew && !DriverStation.isAutonomous()) {
      processVision();
    }
  }

  public void processVision() {
    // create a new pose based off the new inputs
    Pose2d currentPose = new Pose2d(inputs.x, inputs.y, new Rotation2d(inputs.rotation));
    Logger.getInstance().recordOutput(" pose", currentPose);

    // add the new pose to a list
    new PoseAndTimestamp(currentPose, inputs.timestamp);
  }

  public void setReferencePose(Pose2d pose) {
    io.setReferencePose(pose);
  }

  public PoseAndTimestamp getPoseAndTimestamp() {
    return results;
  }

  public static class PoseAndTimestamp {
    Pose2d pose;
    double timestamp;

    public PoseAndTimestamp(Pose2d pose, double timestamp) {
      this.pose = pose;
      this.timestamp = timestamp;
    }

    public Pose2d getPose() {
      return pose;
    }

    public double getTimestamp() {
      return timestamp;
    }
  }
}
