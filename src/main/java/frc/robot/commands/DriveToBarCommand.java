// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveToBarCommand extends CommandBase {
  /** Creates a new DriveToBarCommand. */
  public DriveToBarCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hubTargeting);
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.hubTargeting.barPipeline();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotContainer.hubTargeting.barReady()){
      new FollowGyroCommand(0.2);
    System.out.print("yes");}
    else {
      RobotContainer.drivetrain.drive(
      new Translation2d(0.0,0.0),0.0, true);
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
