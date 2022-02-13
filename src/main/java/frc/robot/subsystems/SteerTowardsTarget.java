// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.TargetDetection;

public class SteerTowardsTarget extends CommandBase {

  private Drivetrain m_drivetrain = RobotContainer.drivetrain;
  private Limelight m_limelight = RobotContainer.limelight;
  private Gyro m_gyro = RobotContainer.gyro;
  // minimum chevron detection area (# square pixels)
  public static final double MIN_HEX_DETECTION_AREA = 1000.0;
  public static final double MIN_HEX_VERT_SIZE = 20.0; 

  public TargetDetection GetTargetEstimation() {

      // determine if camera has acquired a target
      boolean detected = RobotContainer.limelight.isTargetPresent();

      // get target side lengths
      public double vert = RobotContainer.limelight.getVerticalSideLength();
      public double hor = RobotContainer.limelight.getHorizontalSideLength();

      // get target area of target (in sq pixels)
      public double area = vert * hor;
  
    }

  /** Creates a new SteerTowardsTarget. */
  public SteerTowardsTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    addRequirements(m_limelight);
    addRequirements(m_gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    TargetDetection target = GetTargetEstimation();

    // get speed to drive towards ball
    double speed = OI.driverController.getLeftY();

    // get angle to target
    double TargetAngle = 0;


    //get limelight pipeline
    if(m_limelight.getPipeline() == 4){
      TargetAngle = RobotContainer.limelight.getHorizontalTargetOffsetAngle();;
    }
    else{
      if (Math.abs(m_gyro.getYaw()) >= 10.0){
      TargetAngle = -m_gyro.getYaw();
      }
      else if(detected == true){
      // get angle to target
      TargetAngle = RobotContainer.limelight.getHorizontalTargetOffsetAngle();;
      }
    }

    // is angle correction positive or negative?
    if (TargetAngle >=0.0)
      // drive towards target
      RobotContainer.drivetrain.drive(translation, rotation, fieldOriented); //TODO: update this to be correct
    else 
      // drive towards target
      RobotContainer.drivetrain.drive(translation, rotation, fieldOriented); //TODO: update this to be correct
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
