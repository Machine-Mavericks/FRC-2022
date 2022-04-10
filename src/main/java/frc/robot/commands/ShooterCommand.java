// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/** THIS COMMAND IS NOT USED. IF YOU ARE TRYING TO CHANGE SHOOTER CODE, GO TO AUTOSHOOTALLCOMMAND */
/** THIS COMMAND IS NOT USED. IF YOU ARE TRYING TO CHANGE SHOOTER CODE, GO TO AUTOSHOOTALLCOMMAND */
/** THIS COMMAND IS NOT USED. IF YOU ARE TRYING TO CHANGE SHOOTER CODE, GO TO AUTOSHOOTALLCOMMAND */
/** THIS COMMAND IS NOT USED. IF YOU ARE TRYING TO CHANGE SHOOTER CODE, GO TO AUTOSHOOTALLCOMMAND */
/** THIS COMMAND IS NOT USED. IF YOU ARE TRYING TO CHANGE SHOOTER CODE, GO TO AUTOSHOOTALLCOMMAND */

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    //RobotContainer.m_shooter.setShooterSpeed(RobotContainer.m_shooter.ChosenSpeed.getDouble(5000.0));
    RobotContainer.lifter.shooting = true;
    SmartDashboard.putBoolean("shooting boolean", true); //RobotContainer.lifter.shooting);
    RobotContainer.m_shooter.setShooterSpeed(RobotContainer.hubTargeting.GetTargetRPM());
    RobotContainer.m_shooter.setShooterAngle(RobotContainer.hubTargeting.GetTargetHoodSetting());
    RobotContainer.lifter.liftBalls();
    RobotContainer.intake.setMotorSpeed(0.35);
    if(OI.shooterFireButton.get()){
      // And the shooter has reached 95% flywheel speed, feed balls
      if((RobotContainer.m_shooter.getShooterSpeed() >= RobotContainer.hubTargeting.GetTargetRPM()*0.95)){
        //If two balls are loaded, then increment the number of shots taken in shotEvaluationCommand by two, if only one is loaded, increment it by one 
        //if (TwoBallsLoaded){RobotContainer.m_shooter.ShotsTaken+=2;}else{RobotContainer.m_shooter.ShotsTaken+=1;}
        //Set the current pose2D for odometry recording purposes
        ShotEvaluationCommand.RobotPose = RobotContainer.odometry.getPose2d();

        //Lift balls
        RobotContainer.lifter.liftBalls();
      }
    } else {
      // If the fire button isn't held don't feed
      RobotContainer.lifter.stopMotor();
    }

    shootTime += 0.02;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // go back to idle speed and set timer to 0 until command starts again
    RobotContainer.m_shooter.setShooterSpeed(RobotContainer.hubTargeting.getShooterIdleSpeed());
    RobotContainer.lifter.stopMotor();
    RobotContainer.intake.setMotorSpeed(0.0);
    RobotContainer.lifter.shooting = false;
    shootTime = 0.0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (shootTime>1000.0);
    return false;
  }
}
