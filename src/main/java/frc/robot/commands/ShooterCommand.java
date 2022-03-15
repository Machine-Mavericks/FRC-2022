// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotContainer;

public class ShooterCommand extends CommandBase {

  private double shootTime = 0.0;

  /** Creates a new ShooterCommand. */
  public ShooterCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.m_shooter.setShooterSpeed(RobotContainer.hubTargeting.DistanceRPM());
    shootTime = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If the fire button is held
    RobotContainer.m_shooter.setShooterSpeed(RobotContainer.m_shooter.ChosenSpeed.getDouble(5000.0));
    RobotContainer.lifter.leaderLifterTalon.set(ControlMode.PercentOutput, -RobotContainer.m_shooter.ChosenIdleSpeed.getDouble(0.5));
    RobotContainer.intake.setMotorSpeed(0.35);
    // if(OI.shooterFireButton.get()){
    //   // And the shooter has reached 95% power, feed balls
    //   if(RobotContainer.m_shooter.getShooterSpeed() >= RobotContainer.hubTargeting.DistanceRPM()*0.95){
    //     RobotContainer.lifter.liftBalls();
    //   }
    // } else {
    //   // If the fire button isn't held don't feed
    //   RobotContainer.lifter.stopMotor();
    // }

    shootTime += 0.02;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // go back to idle speed and set timer to 0 until command starts again
    //RobotContainer.m_shooter.setShooterSpeed(RobotContainer.m_shooter.ChosenIdleSpeed.getDouble(2500));
    //RobotContainer.lifter.stopMotor(); TODO: put back
    shootTime = 0.0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shootTime>5000.0);
    //return false;
  }
}
