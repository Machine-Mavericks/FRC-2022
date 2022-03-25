// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ClimbCommand extends CommandBase {
  
  int motorSpeed;

  /** Creates a new ClimbCommand. */
  public ClimbCommand(boolean reverse) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
    boolean m_reverse = reverse;

    if (m_reverse)
      motorSpeed = 600;
    else
      motorSpeed = -600;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climber.motorVelocity(motorSpeed);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.motorVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
