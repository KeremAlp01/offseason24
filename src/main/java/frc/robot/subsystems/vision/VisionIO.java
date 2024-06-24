// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public double x;
    public double y;
    public double rotation;
    public Pose2d robotPose;
    public Pose2d currentPose;
    public double timestamp;
    public boolean isNew;
    public boolean hasTargets = false;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default Pose2d getEstimatedPose() {
    return new Pose2d();
  }

  public default void setReferencePose(Pose2d pose) {}
}
