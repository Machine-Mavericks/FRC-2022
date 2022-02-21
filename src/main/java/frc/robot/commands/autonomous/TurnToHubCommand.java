// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

/**
 * Command which turns the robot at a provided speed until the hub has been detected
 */
public class TurnToHubCommand extends CommandBase {
  double turnSpeed;
  /** Creates a new TurnToHubCommand. */
  public TurnToHubCommand(double turnSpeed) {
    addRequirements(RobotContainer.drivetrain);
    this.turnSpeed = turnSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
