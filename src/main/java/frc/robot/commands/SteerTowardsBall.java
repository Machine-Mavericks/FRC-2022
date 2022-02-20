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

public class SteerTowardsBall extends CommandBase {

  private Drivetrain m_drivetrain = RobotContainer.drivetrain;
  private Gyro m_gyro = RobotContainer.gyro;

  // get angle to target
  double TargetAngle = 0;

  // TODO: set gains
  double kp = 0.0125;
  double ki = 0.0;
  double kd = 0.0; //0.00015;

  // is this command automated or semi-automated?
  boolean m_automated;

  PIDController pidController = new PIDController(kp, ki, kd);

  /** Creates a new SteerTowardsTarget.
   * Input: true if fully automated, false if only sem-automated
   */
  public SteerTowardsBall(boolean automated) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    addRequirements(m_gyro);
    RobotContainer.ballTargeting.setBallPipeline();

    // set automation flag
    m_automated = automated;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double yInput = 0.0; //= OI.driverController.getLeftY()*0.2;
    double xInput = 0.0; //= OI.driverController.getLeftX()*0.2;

    double angle = 0.0;

    // do we have a valid target?
    if ((RobotContainer.ballTargeting.IsBall())){

      TargetAngle = RobotContainer.ballTargeting.ballAngle();
    
      // determine angle correction - uses PI controller
      angle = pidController.calculate(TargetAngle);
      if (angle > 1.0)
        angle = 1.0;
      if (angle < -1.0)
        angle = -1.0;

      // if not fully automatic, get joystick inputs
      if (!m_automated)
      {
        yInput = OI.driverController.getLeftY()*0.2;
        xInput = OI.driverController.getLeftX()*0.2;
      }
      
      /*// is angle correction positive or negative?
      if (TargetAngle >= 0.0) {
        // drive towards target
        RobotContainer.drivetrain.drive(
            new Translation2d(yInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                xInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
                angle * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true);
      } // TODO: update this to be correct
      else {
        // drive towards target
        RobotContainer.drivetrain.drive(
            new Translation2d(yInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                xInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
                angle * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true);
      } // TODO: update this to be correct
*/
    
    }   // end if we have a valid target
    
  yInput = 0.5;
  RobotContainer.intake.setMotorSpeed(0.5);

  RobotContainer.drivetrain.drive(
    new Translation2d(yInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
        xInput * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
        angle * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setMotorSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
