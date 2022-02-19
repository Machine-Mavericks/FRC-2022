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
    // Driver inputs, should be in range [-1,1]
    // TODO: Map to controller, using functions like OI.driverController.getLeftX()
    double xInput = OI.driverController.getLeftX();
    double yInput =   OI.driverController.getLeftY();
    double rotInput = OI.driverController.getRightX()*0.25;

    double maxAccel = RobotContainer.drivetrain.maxAccel.getDouble(0.02);

    xInput = Math.abs(RobotContainer.drivetrain.MaxAccelX(maxAccel)) > 0.1 ? RobotContainer.drivetrain.MaxAccelX(maxAccel) : 0;
    yInput = Math.abs(RobotContainer.drivetrain.MaxAccelY(maxAccel)) > 0.1 ? RobotContainer.drivetrain.MaxAccelY(maxAccel) : 0;
    rotInput = Math.abs(rotInput) > 0.1 ? rotInput*0.25 : 0;

    m_drivetrain.drive(new Translation2d(yInput*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, xInput*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND), rotInput*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true); 
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
