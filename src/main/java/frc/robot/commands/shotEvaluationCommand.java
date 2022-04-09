// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.RobotContainer;

public class shotEvaluationCommand extends CommandBase {
  //int ShotNumber;
  
  private double ShooterSpeedOffset = 0;

  public static int ShotsTaken = 0;
  public static Pose2d RobotPose;
  private int ShotsLogged = 0;

  private String[] LastTwoShots = {"",""};

  private final double RPMIncrement = 0.05;

  //Logs odometry and shot successfullness, but does nothing with it, implement logging to file later.
  private ArrayList<ArrayList<String>> ShotList = new ArrayList<ArrayList<String>>();

  /** Creates a new shotEvaluationCommand. */
  public shotEvaluationCommand(String shotType) {
    if (ShotsTaken > ShotsLogged){
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
      ShotList.add(SavedCurrentShotData);

      //Move previous shot back by one in array
      LastTwoShots[0] = LastTwoShots[1];
      //Set new shot into array
      LastTwoShots[1] = shotType;

      ShotsLogged+=1;
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
    if (LastTwoShots[0]==LastTwoShots[1]) {
      switch (LastTwoShots[1]) {
        case "Hit":
          Offset = 0;
        case "Overshoot":
          Offset = RPMIncrement;
        case "Undershoot":
          Offset = -RPMIncrement;
        default:
          Offset = 0;
      }
      ShooterSpeedOffset+=Offset;
      //RobotContainer.hubTargeting.m_OnTheFlyRPMAdjust = ShooterSpeedOffset;
      RobotContainer.hubTargeting.m_DistanceAdjust.setDouble(ShooterSpeedOffset);
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
