// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class RetractClimber extends CommandBase {
  int m_righttargetpos;
  //int m_lefttargetpos;
  
  /** Creates a new RetractClimber. */
  public RetractClimber() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // get starting position
    m_righttargetpos = RobotContainer.climber.getRightMotorPosition();
    //m_lefttargetpos = RobotContainer.climber.getLeftMotorPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // decrement calculation
    // 7 turns * 48 ratio * 2048 counts/turn / 10 seconds / 50 Hz
    // = 1,376 counts/interation
    
    // decrement our target
    // if arms currently at same target
    // if (m_righttargetpos == m_lefttargetpos)
    // {
      m_righttargetpos -= 4000; //1376;
      //m_lefttargetpos -= 4000; //1376;
    // }
    // else
    // {
    //   // we are not in unison - even up the arms
    //   if (m_lefttargetpos > m_righttargetpos)
    //     m_lefttargetpos -= Math.min(m_lefttargetpos-m_righttargetpos, 1376);
    //   else if (m_lefttargetpos < m_righttargetpos)
    //     m_righttargetpos -= Math.min(m_righttargetpos - m_lefttargetpos, 1376);
    // }
    
    // limit target to end of position
    if (m_righttargetpos <=0)
      m_righttargetpos = 0;
    // if (m_lefttargetpos <= 0)
    //   m_lefttargetpos = 0;
    
    // command climbers to move
    //RobotContainer.climber.setLeftMotorPosition(m_lefttargetpos);
    RobotContainer.climber.setRightMotorPosition(m_righttargetpos);

    //RobotContainer.climber.setLeftMotorPosition(0);
    //RobotContainer.climber.setRightMotorPosition(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_righttargetpos <=0);
  }
}
