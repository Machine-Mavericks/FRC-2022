// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimbCommand extends CommandBase {
  
  private double distanceToDrive;
  private double m_climbTime;
  private double m_targetClimbTime;
  private String m_targetClimbLevel;

  private double m_positiontolerance = 0.10;
  private double m_angletolerance = 6.0;
  private double m_timeout;

  private Pose2d m_target;
  private double m_speed = 0.2;
  private double m_rotspeed = 0.1;
  private Rotation2d m_targetRotation;

  private double m_time;

  // x, y, rotation PID controllers
  private PIDController m_xController = new PIDController(0.50, 0, 0.035);
  private PIDController m_yController = new PIDController(0.50, 0, 0.035);
  private PIDController m_rotController = new PIDController(0.01, 0, 0.001);

  /** Creates a new ClimbCommand. */
  public ClimbCommand(String targetClimbLevel) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
    addRequirements(RobotContainer.hubTargeting);
    addRequirements(RobotContainer.drivetrain);
    addRequirements(RobotContainer.odometry);
    m_targetClimbLevel = targetClimbLevel;
  }

  public static double AngleDifference( double angle1, double angle2 )
  {
      double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
      return diff < -180 ? diff + 360 : diff;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_targetRotation = RobotContainer.odometry.getPose2d().getRotation();

       // calculate the distance we need to drive based on the bar target
       if (m_targetClimbLevel == "low"){
        distanceToDrive = RobotContainer.hubTargeting.estimateHorizontalDistanceToLowerBar();
      }
      else
        distanceToDrive = RobotContainer.hubTargeting.estimateHorizontalDistanceToUpperBar();
      
      double currentX = RobotContainer.odometry.getPose2d().getX();
      double currentY = RobotContainer.odometry.getPose2d().getY();
      
      // drive towards the bar
      m_target = new Pose2d(new Translation2d(currentX - distanceToDrive, 
      currentY), m_targetRotation);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climbTime +=0.02;
    // if (DriverStation.getMatchTime() < 30.0) //do not deploy climber if it isn't endgame
    //   RobotContainer.climber.motorVelocity();
    // else
    // RobotContainer.climber.m_climberFalcon.set(ControlMode.PercentOutput, 0.0);

    ////////////////////////////////////////////

    Pose2d curr = RobotContainer.odometry.getPose2d();
    
    // increment time
    m_time += 0.02;

    // execute PIDs
    double xSpeed = m_xController.calculate(curr.getX()- m_target.getX() );
    double ySpeed = -m_yController.calculate(curr.getY() - m_target.getY() );
    
    double rotSpeed = m_rotController.calculate(-AngleDifference(curr.getRotation().getDegrees(),m_target.getRotation().getDegrees()));
    //double rotSpeed = m_rotController.calculate(curr.getRotation().getDegrees() - m_target.getRotation().getDegrees());

    // limit speeds to allowable
    if (xSpeed > m_speed)
      xSpeed = m_speed;
    if (xSpeed < -m_speed)
      xSpeed = -m_speed;
    if (ySpeed > m_speed)
      ySpeed = m_speed; 
    if (ySpeed < -m_speed)
      ySpeed = -m_speed;  
    if (rotSpeed >m_rotspeed)
      rotSpeed = m_rotspeed;
    if (rotSpeed < -m_rotspeed)
      rotSpeed = -m_rotspeed;

    // drive robot according to x,y,rot PID controller speeds
    RobotContainer.drivetrain.drive(new Translation2d(xSpeed*RobotContainer.drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                                                      ySpeed*RobotContainer.drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
                                    rotSpeed*RobotContainer.drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                                    true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.m_climberFalcon.set(ControlMode.PercentOutput, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d curr = RobotContainer.odometry.getPose2d();

    // we are finished if we are within erorr of target or command had timed out
    return (((Math.abs(m_target.getX() - curr.getX()) <  m_positiontolerance) &&
          (Math.abs(m_target.getY() - curr.getY()) <  m_positiontolerance) &&
          (Math.abs(m_target.getRotation().getDegrees() - curr.getRotation().getDegrees()) < m_angletolerance)) ||
          (m_time >= m_timeout));
  }
}
