// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimbCommand extends CommandBase {
  
  double motorSpeed;
  double m_climbTime;
  double m_targetClimbTime;
  String m_targetClimbLevel;

  /** Creates a new ClimbCommand. */
  public ClimbCommand(int targetClimbTime, String targetClimbLevel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
    addRequirements(RobotContainer.hubTargeting);
    addRequirements(RobotContainer.drivetrain);
    addRequirements(RobotContainer.odometry);
    m_targetClimbTime = targetClimbTime;
    m_targetClimbLevel = targetClimbLevel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbTime = 0.0;
    double distanceToDrive;

    if (m_targetClimbLevel == "low"){
      distanceToDrive = RobotContainer.hubTargeting.estimateHorizontalDistanceToLowerBar();
    }
    else
      distanceToDrive = RobotContainer.hubTargeting.estimateHorizontalDistanceToLowerBar();
    
    
    double currentX = RobotContainer.odometry.getPose2d().getX();
    double currentY = RobotContainer.odometry.getPose2d().getY();
    
    new AutoDriveToPose(new Pose2d(new Translation2d(currentX - distanceToDrive, 
    currentY), 
    Rotation2d.fromDegrees(-90.0)), 
    0.7, 0.10, 15.0);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climbTime +=0.02;
    if (DriverStation.getMatchTime() < 30.0) //do not deploy climber if it isn't endgame
      RobotContainer.climber.motorVelocity();
    else
    RobotContainer.climber.m_climberFalcon.set(ControlMode.PercentOutput, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.m_climberFalcon.set(ControlMode.PercentOutput, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_climbTime>=m_targetClimbTime); 
  }
}
