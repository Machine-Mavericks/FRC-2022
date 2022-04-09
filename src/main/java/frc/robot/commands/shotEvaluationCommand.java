// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShotEvaluationCommand extends CommandBase {

  public static Pose2d RobotPose;

  Shooter m_shooter = RobotContainer.m_shooter;
  public final double RPMIncrement = 30;

  /** Creates a new shotEvaluationCommand. */
  public ShotEvaluationCommand(String shotType) {
    if (m_shooter.ShotsTaken > m_shooter.ShotsLogged){
      //Define arraylist of data for this shot
      ArrayList<String> SavedCurrentShotData = new ArrayList<String>();
      
      //Define the robot's current Pose2D
      //Pose2d RobotPose = RobotContainer.odometry.getPose2d();

      //First value is what happened to that shot
      SavedCurrentShotData.add(shotType);
      //Second value is X position
      SavedCurrentShotData.add(String.valueOf(RobotPose.getX()));
      //Third value is Y position
      SavedCurrentShotData.add(String.valueOf(RobotPose.getY()));
      //Fourth value is robot rotation in degrees
      SavedCurrentShotData.add(String.valueOf(RobotPose.getRotation().getDegrees()));

      //Save this shot to the list of shots
      m_shooter.ShotList.add(SavedCurrentShotData);

      //Move previous shot back by one in array
      m_shooter.LastTwoShots[0] = m_shooter.LastTwoShots[1];
      //Set new shot into array
      m_shooter.LastTwoShots[1] = shotType;

      m_shooter.ShotsLogged+=1;
    }
   

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Offset;
    if (m_shooter.LastTwoShots[0]==m_shooter.LastTwoShots[1]) {
      switch (m_shooter.LastTwoShots[1]) {
        case "Hit":
          Offset = 0;
        case "Overshoot":
          Offset = RPMIncrement;
        case "Undershoot":
          Offset = -RPMIncrement;
        default:
          Offset = 0;
      }
      m_shooter.ShooterSpeedOffset+=Offset;
      RobotContainer.hubTargeting.m_OnTheFlyRPMAdjust = m_shooter.ShooterSpeedOffset;
    }
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
