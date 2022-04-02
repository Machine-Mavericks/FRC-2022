// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class shotEvaluationCommand extends CommandBase {
  /** Creates a new shotEvaluationCommand. */
  int underShots;
  int overShots;
  public boolean shotWasShort;
  public double shooterAdjustFactor;

  public shotEvaluationCommand(boolean underShoot) {
    // Use addRequirements() here to declare subsystem dependencies.
    shotWasShort = underShoot;
    addRequirements(RobotContainer.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shotWasShort)
      underShots +=1;
    else
      overShots +=1;
    
    if (overShots - underShots == 3)
      shooterAdjustFactor += 1;
    if (underShots- overShots ==3)
      shooterAdjustFactor -=1;

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
