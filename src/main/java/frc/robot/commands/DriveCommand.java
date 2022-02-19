// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotContainer;

public class DriveCommand extends CommandBase {

  private Drivetrain m_drivetrain;

  /** Creates a new DriveCommand. */
  public DriveCommand(Drivetrain drivetrain) {
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(new Translation2d(OI.getYDriveInput()*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, OI.getXDriveInput()*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND), OI.getRotDriveInput()*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns false when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
