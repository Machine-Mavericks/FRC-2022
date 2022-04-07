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
  double kd = 0.0; // 0.00015;

  PIDController pidController = new PIDController(kp, ki, kd);

  double angle;
  double m_speed;
  double targetYaw;
  double err;
  double largestTy;
  boolean barSeen;
  int m_climbLevel;
  boolean m_raiseClimb;

  public FollowGyroCommand(int climbLevel, boolean raiseClimb) {
    m_climbLevel = climbLevel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
    addRequirements(RobotContainer.gyro);
    addRequirements(RobotContainer.hubTargeting);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetYaw = RobotContainer.gyro.getYaw();
    RobotContainer.hubTargeting.barPipeline();
    largestTy = RobotContainer.hubTargeting.getBarTY();
    barSeen = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!m_raiseClimb)
      m_speed = 0.1;

    if (!barSeen && RobotContainer.hubTargeting.getBarTY() != 0)
      barSeen = true;

    if (m_climbLevel == 4) {
      if (m_raiseClimb){
      if (!barSeen)
        m_speed = 0.5;
      if (RobotContainer.hubTargeting.getBarTY() > -18.0 && RobotContainer.hubTargeting.getBarTY() < -15.0)
        m_speed = 0.2;
      if (RobotContainer.hubTargeting.getBarTY() > -15.0)
        m_speed = 0.08;
      if (!(RobotContainer.hubTargeting.getBarTY() > -10 && RobotContainer.hubTargeting.getBarTY() != 0)) {
        if (RobotContainer.hubTargeting.getBarTY() > largestTy)
          largestTy = RobotContainer.hubTargeting.getBarTY();
        err = RobotContainer.gyro.getYaw() - targetYaw;
        angle = pidController.calculate(err);
        RobotContainer.drivetrain.drive(
            new Translation2d(-m_speed * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.0),
            angle * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false);
      }}
    }

    else if (m_climbLevel == 6) {
      
      if (!barSeen)
      // decrease the speed as we approach the bar
        m_speed = 0.4;
      if (RobotContainer.hubTargeting.getBarTY() > -18.0 && RobotContainer.hubTargeting.getBarTY() <= 5.0)
        m_speed = 0.3;
      if (RobotContainer.hubTargeting.getBarTY() > 5.0)
        m_speed = 0.1;

      // drive if we have not yet met our ty threshold or if the target is not present
      if (!(RobotContainer.hubTargeting.getBarTY() > 20 && RobotContainer.hubTargeting.getBarTY() != 0)) {
        
        // keep track of largest ty
        if (RobotContainer.hubTargeting.getBarTY() > largestTy)
          largestTy = RobotContainer.hubTargeting.getBarTY();

        // get the angle error for the gyro
        err = RobotContainer.gyro.getYaw() - targetYaw;
        angle = pidController.calculate(err);

        // drive following the gyro
        RobotContainer.drivetrain.drive(
            new Translation2d(-m_speed * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.0),
            angle * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false);
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.drivetrain.drive(new Translation2d(0.0, 0.0), 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_climbLevel == 4)
      return (RobotContainer.hubTargeting.getBarTY() > -7 && RobotContainer.hubTargeting.getBarTY() != 0);
    else
      return (RobotContainer.hubTargeting.getBarTY() > 20 && RobotContainer.hubTargeting.getBarTY() != 0);
  }
}
