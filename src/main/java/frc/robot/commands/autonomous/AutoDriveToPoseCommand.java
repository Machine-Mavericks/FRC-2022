// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoDriveToPoseCommand extends CommandBase {
  
  private Pose2d m_target;
  private double m_speed;

  private PIDController m_xController = new PIDController(1, 0, 0);
  private PIDController m_yController = new PIDController(1, 0, 0);
  private PIDController m_rotController = new PIDController(1, 0, 0);

  
  /** Creates a new AutoDriveToPoseCommand. */
  public AutoDriveToPoseCommand(Pose2d target, double speed) {
    addRequirements(RobotContainer.drivetrain);
    this.m_target = target;
    this.m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d curr = RobotContainer.odometry.getPose2d();
    // Translation2d err = m_target.relativeTo(other)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new Translation2d(0, 0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
