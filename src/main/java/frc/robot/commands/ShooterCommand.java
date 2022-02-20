// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ShooterCommand extends CommandBase {

  private static final double SPEED_TOLERANCE = 5;

  /** Creates a new ShooterCommand. */
  public ShooterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set shooter speed to that set on shuffleboard
    RobotContainer.m_shooter.setShooterSpeed(RobotContainer.m_shooter.ChosenSpeed.getDouble(5000.0));
    if(Math.abs(RobotContainer.m_shooter.getShooterSpeed() - RobotContainer.m_shooter.ChosenSpeed.getDouble(5000.0)) < RobotContainer.m_shooter.speedTolerance.getDouble(5)){
      RobotContainer.lifter.liftBalls();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // go back to idle speed and set timer to 0 until command starts again
    RobotContainer.m_shooter.setShooterSpeed(RobotContainer.m_shooter.ChosenIdleSpeed.getDouble(2500));
    RobotContainer.lifter.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
