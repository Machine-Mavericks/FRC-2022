// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

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
      System.out.println("working");
      RobotContainer.drivetrain.drive(
        new Translation2d(-0.2 * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
            0.0 * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
            0.0 * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true); }
    else {
      RobotContainer.drivetrain.drive(
      new Translation2d(0.0 * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
          0.0 * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
          0.0* Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true);
      System.out.println("not working");
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
