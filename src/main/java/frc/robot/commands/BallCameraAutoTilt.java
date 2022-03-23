// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class BallCameraAutoTilt extends CommandBase {
  
  double m_notargettimer;

  // PID gains for rotating robot towards ball target
  double kp = 0.22;
  double ki = 0.0;
  double kd = 0.0;
  
  // camera tilt controller
  PIDController pidController = new PIDController(kp, ki, kd);


  /** Creates a new BallCameraAutoTilt. */
  public BallCameraAutoTilt() {
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(RobotContainer.cameraTilt);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_notargettimer=0.0;
    pidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double adjust = 0.0;

    if (RobotContainer.ballTargeting.IsBall())
    {
      adjust = -pidController.calculate(RobotContainer.ballTargeting.getBallVertAngle());
      if (Math.abs(adjust) <= 0.5)
        adjust = 0.0;
      RobotContainer.cameraTilt.setAngle(RobotContainer.cameraTilt.getAngle() + adjust);  
      m_notargettimer = 0.0;

      // if we are 'looking down', widen ball detection to look for non-square objects
      if (RobotContainer.cameraTilt.getAngle() < -30.0)
      {
        if (DriverStation.getAlliance() == Alliance.Red)
          RobotContainer.ballTargeting.setBallPipeline(3);
        else
          RobotContainer.ballTargeting.setBallPipeline(4);
      }
      else
      {
        if (DriverStation.getAlliance() == Alliance.Red)
          RobotContainer.ballTargeting.setBallPipeline(1);
        else
          RobotContainer.ballTargeting.setBallPipeline(2);
      }
    }
    
    // we have no target. Increment timer
    else
      {m_notargettimer += 0.02;
      pidController.reset();
      }
    
    
    // we have no target - raise camera
    // reset pipeline to ordinary ball detection
    if (m_notargettimer > 1.0)
    {
      RobotContainer.cameraTilt.setAngle (-7.4);  // was -14.4
      if (DriverStation.getAlliance() == Alliance.Red)
        RobotContainer.ballTargeting.setBallPipeline(1);
      else
        RobotContainer.ballTargeting.setBallPipeline(2);
    }
  }

  /** forces camera to look low for a ball */
  public void forceCameraLow()
  {
    m_notargettimer = 0.0;
    RobotContainer.cameraTilt.setAngle (-39.6);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
