// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.DoubleSupplier;

public class FlywheelCommand extends Command {
  private final Flywheel shooter;
  private final DoubleSupplier rpm;
  /** Creates a new FlywheelCommand. */
  public FlywheelCommand(DoubleSupplier rpm, Flywheel shooter) {
    this.rpm = rpm;
    this.shooter = shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runVelocity(rpm.getAsDouble());
    System.out.println(rpm.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.runVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
