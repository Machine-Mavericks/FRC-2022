// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotContainer;

public class ShooterCommand extends CommandBase {

  /** Creates a new ShooterCommand. */
  public ShooterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_shooter.setShooterSpeed(RobotContainer.hubTargeting.DistanceRPM());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If the fire button is held
    if(OI.shooterFireButton.get()){
      // And the shooter has reached 95% power, feed balls
      if(RobotContainer.m_shooter.getShooterSpeed() >= RobotContainer.hubTargeting.DistanceRPM()*0.95){
        RobotContainer.lifter.liftBalls();
      }
    } else {
      // If the fire button isn't held don't feed
      RobotContainer.lifter.stopMotor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // go back to idle speed and set timer to 0 until command starts again
    //RobotContainer.m_shooter.setShooterSpeed(RobotContainer.m_shooter.ChosenIdleSpeed.getDouble(2500));
    RobotContainer.lifter.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
