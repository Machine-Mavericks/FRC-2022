// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeCommand extends CommandBase {
  private int m_timer = 0;
  // time it will take until the intake retracts, set at 5 seconds currently
  private static final int END_TIME_TICKS = 5 * 50;

  /** Creates a new IntakeCommand. */
  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intake.extend();
    m_timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.retract();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer >= END_TIME_TICKS) {
      return true;
    } else {
      return false;
    }
  }
}
