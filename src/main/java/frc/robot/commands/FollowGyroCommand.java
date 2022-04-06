// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class FollowGyroCommand extends CommandBase {
  /** Creates a new FollowGyroCommand. */

    double kp = 0.0125;
    double ki = 0.00001;
    double kd = 0.0; //0.00015;
  
    PIDController pidController = new PIDController(kp, ki, kd);

    double angle;
    double m_speed;
    double targetYaw;
    double err;

  public FollowGyroCommand(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
    addRequirements(RobotContainer.gyro);
    m_speed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetYaw = RobotContainer.gyro.getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    err = RobotContainer.gyro.getYaw() - targetYaw;
    angle = pidController.calculate(err);
    RobotContainer.drivetrain.drive(
            new Translation2d(-m_speed * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.0),
                angle * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new Translation2d(0.0,0.0),0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (false);
  }
}
