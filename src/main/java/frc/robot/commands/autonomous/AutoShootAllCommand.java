// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.math.geometry.Translation2d;


public class AutoShootAllCommand extends CommandBase {
  
  private double m_time;
  private double m_noballtime;
  private double m_startdelay = 0.3;
  private double m_enddelay = 0.1;
  private double m_timeout = 2.0;
  
  /** Creates a new AutoShootAllCommand. */
  public AutoShootAllCommand() {
    // aiming uses drive system
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time = 0.0;
    m_noballtime = 0.0;

    // stop robot from moving
    RobotContainer.drivetrain.drive(new Translation2d(0,0),0, false);

    // set our shooting solution
    RobotContainer.m_shooter.setShooterAngle(RobotContainer.hubTargeting.GetTargetHoodSetting());
    RobotContainer.m_shooter.setShooterSpeed(RobotContainer.hubTargeting.GetTargetRPM());
    RobotContainer.m_shooter.setTopShooterSpeed(RobotContainer.hubTargeting.GetTopTargetRPM());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_time += 0.02;
    
    // is it time to switch on lifter? turn on lifter after initial delay
    if (m_time > m_startdelay && RobotContainer.hubTargeting.isTargetPresent())
      // lift balls - set lifter motor to 5,000 rpm
      RobotContainer.lifter.liftBalls(-2500.0);

    // do we have any balls in the stack? if not, count up no-ball time
    // if we find something, then restart time counter
    if ((!RobotContainer.intake.GetIntakeLimitSwitchStatus()) && RobotContainer.lifter.liftLimit.get())
      m_noballtime += 0.02;
    else
      m_noballtime = 0.0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // reset shooter back to idle, and stop lifter
    //RobotContainer.m_shooter.setShooterSpeed(RobotContainer.hubTargeting.getShooterIdleSpeed());
    RobotContainer.lifter.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if we have not had any balls for certain time, then we have finished
    // if time out reached, then consider command completed
    return ((m_noballtime >= m_enddelay) || (m_time > m_timeout));
  }
}
