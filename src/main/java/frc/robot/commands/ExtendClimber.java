// Copy (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ExtendClimber extends CommandBase {
  int m_targetpos;
  //int m_lefttargetpos;
  
  /** Creates a new ExtendClimber. */
  public ExtendClimber() {
    // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(RobotContainer.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
      // get starting position
      m_targetpos = RobotContainer.climber.getMotorPosition();
      //m_lefttargetpos = RobotContainer.climber.getLeftMotorPosition();
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // increment calculation
    // 7 turns * 48 ratio * 2048 counts/turn / 10 seconds / 50 Hz
    // = 1,376 counts/interation
    
    // increment our target
    // if arms currently at same target
    // if (m_targetpos == m_lefttargetpos)
    // {
      m_targetpos += 5000; //1376;
      //m_lefttargetpos += 5000; //1376;
    // }
    // else
    // {
    //   // we are not in unison - even up the arms
    //   if (m_lefttargetpos < m_targetpos)
    //     m_lefttargetpos += Math.min(m_targetpos - m_lefttargetpos, 1376);
    //   else if (m_lefttargetpos > m_targetpos)
    //     m_targetpos += Math.min(m_lefttargetpos - m_targetpos, 1376);
    // }
    
    // limit target to end of position
    if (m_targetpos >= (int)(8.80*48*2048.0))
      m_targetpos = (int)(8.80*48*2048.0);
    // if (m_lefttargetpos >= (int)(8.80*48*2048))
    //   m_lefttargetpos = (int)(8.80*48*2048.0);

    // command climbers to move
    //RobotContainer.climber.setLeftMotorPosition(m_lefttargetpos);
    RobotContainer.climber.setMotorPosition(m_targetpos);
    
    //RobotContainer.climber.setLeftMotorPosition(7*48*2048);
    //RobotContainer.climber.setMotorPosition(7*48*2048);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_targetpos >= (int)(8.80*48*2048));
  }
}
