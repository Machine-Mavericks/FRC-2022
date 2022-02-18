// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Limelight;

public class SteerTowardsBall extends CommandBase {

  private Drivetrain m_drivetrain = RobotContainer.drivetrain;
  private Limelight m_limelight = RobotContainer.limelight;
  private Gyro m_gyro = RobotContainer.gyro;

  // get angle to target
  double TargetAngle = 0;

  // TODO: set gains
  double kp = -0.002;
  double ki = 0.0;
  double kd = 0.00;

  PIDController pidController = new PIDController(kp, ki, kd);

  /** Creates a new SteerTowardsTarget. */
  public SteerTowardsBall() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    addRequirements(m_limelight);
    addRequirements(m_gyro);
    RobotContainer.ballTargeting.setBallPipeline();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if ((RobotContainer.ballTargeting.IsBall())){

      TargetAngle = RobotContainer.ballTargeting.ballAngle();

      double angle = pidController.calculate(TargetAngle);

      // get speed to drive towards ball
      double yInput = OI.driverController.getLeftY()*0.2;
      double xInput = OI.driverController.getLeftX()*0.2;

      // is angle correction positive or negative?
      if (TargetAngle >= 0.0) {
        // drive towards target
        RobotContainer.drivetrain.drive(
            new Translation2d(yInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                xInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
                angle * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false);
      } // TODO: update this to be correct
      else {
        // drive towards target
        RobotContainer.drivetrain.drive(
            new Translation2d(yInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                xInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
                angle * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false);
      } // TODO: update this to be correct

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}