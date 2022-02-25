// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/**
 * Drive to specified distance from the hub
 * Note: This assumes the hub is already in view
 */
public class AutoHubDistanceCommand extends CommandBase {

  private static final double kPAngle = -0.04;
  private static final double kPDist = -1.5; 

  double targetDistance;
  double tolerance;

  /** Creates a new AutoHubDistanceCommand. */
  public AutoHubDistanceCommand(double targetDistance, double tolerance) {
    addRequirements(RobotContainer.drivetrain);
    this.targetDistance = targetDistance;
    this.tolerance = tolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if ((RobotContainer.hubTargeting.IsTarget())){
      double angleErr = RobotContainer.hubTargeting.getHubAngle();
      double distErr = RobotContainer.hubTargeting.EstimateDistance() - targetDistance;
      RobotContainer.drivetrain.drive(new Translation2d(distErr * kPDist, 0), angleErr*kPAngle, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new Translation2d(0,0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Return when target not found or distance within tolerance
    return !RobotContainer.hubTargeting.IsTarget() || Math.abs(RobotContainer.hubTargeting.EstimateDistance() - targetDistance) < 0.25;
  }
}
