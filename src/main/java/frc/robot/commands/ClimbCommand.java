// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ClimbCommand extends CommandBase {
  
  double motorSpeed;

  double targetTy = 0.0;

  double kp= 0.01;
  double ki= 0.0;
  double kd= 0.0;

  PIDController pidController = new PIDController(kp,ki,kd);

  /** Creates a new ClimbCommand. */
  public ClimbCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
    addRequirements(RobotContainer.ballTargeting);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the channel to look for the line
    //RobotContainer.ballTargeting.setBallPipeline(6);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (RobotContainer.ballTargeting.isTarget()){
    // double err = RobotContainer.ballTargeting.getBallVertAngle() - targetTy;

    // double speed = pidController.calculate(err);

    // //limit our speed
    // if (speed > 0.1)
    //   speed = 0.1;
    // if (speed < -0.1)
    //   speed = -0.1;

    // RobotContainer.drivetrain.drive(new Translation2d(speed*RobotContainer.drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
    //                                                   0.0), 0.0,true);
    // }

    // if (DriverStation.getMatchTime() < 30.0) //do not deploy climber if it isn't endgame
    //   RobotContainer.climber.motorVelocity();
    // else
    // RobotContainer.climber.m_climberFalcon.set(ControlMode.PercentOutput, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.m_climberFalcon.set(ControlMode.PercentOutput, 0.0);
    RobotContainer.drivetrain.drive(new Translation2d(0.0,0.0), 0.0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}
