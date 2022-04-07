// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class shotEvaluationCommand extends CommandBase {
  /** Creates a new shotEvaluationCommand. */
  //int ShotNumber;
  
  private double ShooterSpeedOffset = 0;
  private int Overshoots = 0;
  private int Undershoots = 0;
  private int Bounceouts = 0;
  private int Hits = 0;

  private static double RPMIncrement = 100;

  public shotEvaluationCommand(String shotType) {
    //What kind of shot just happened?
    switch (shotType) {
      case "Hit":
        Hits+=1;
      case "Overshoot":
        Overshoots+=1;
      case "Undershoot":
        Undershoots+=1;
      case "Bounced-Out":
        Bounceouts+=1;
      default:
        Hits+=1;
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
    double Offset = Undershoots - Overshoots;
    Offset = Offset * RPMIncrement;
    
    //If the RPM offset needed has changed, print to log what it changed to, move this all to shuffleboard later.
    if (Offset != ShooterSpeedOffset){
      ShooterSpeedOffset = Offset;
      System.out.println("Changing shooter RPM offset to: " + ShooterSpeedOffset);
    }

    RobotContainer.hubTargeting.m_OnTheFlyRPMAdjust = ShooterSpeedOffset;
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
