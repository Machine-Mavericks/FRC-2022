// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

/**
 * Command which turns the robot at a provided speed until the hub has been detected
 */
public class TurnToHubCommand extends CommandBase {
  double turnSpeed;
  /** 
   * Creates a new TurnToHubCommand. 
   * @param turnSpeed Speed to make turn, in %{@link Drivetrain#MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND}
   * @param timeout Timeout in seconds
  */
  public TurnToHubCommand(double turnSpeed, double timeout) {
    addRequirements(RobotContainer.drivetrain);
    this.withTimeout(timeout);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Determine direction from odometry
    Pose2d pose = RobotContainer.odometry.getPose2d();
    double targetAngle = 0;
    if(pose.getX() < 6.25 && pose.getY() < 3.125) {
      // If in near left
      targetAngle = 45;
    } else if (pose.getX() > 6.25 && pose.getY() < 3.125){
      // If in far left
      targetAngle = -45+360;
    } else if (pose.getX() < 6.25 && pose.getY() > 3.125){
      // If in near right
      targetAngle = 135;
    } else {
      // If in far right
      targetAngle = -135+360;
    }
    // Get angle, adjust coordiante system to simplify math
    double currAngle = RobotContainer.odometry.getAngle();
    if(currAngle < 0) currAngle += 360;
    // Use sin to determine optimal direction
    if(Math.sin(currAngle-targetAngle) < 0) {
      turnSpeed = -turnSpeed;
    }
    // Start driving desired direction
    RobotContainer.drivetrain.drive(new Translation2d(0,0), turnSpeed*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new Translation2d(0,0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End when the hub has been detected
    return RobotContainer.hubTargeting.IsTarget();
  }
}
