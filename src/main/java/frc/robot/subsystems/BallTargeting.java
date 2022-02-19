// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallTargeting extends SubsystemBase {
  private Limelight m_ballCamera;

  /** Creates a new HubTargeting. */
  public BallTargeting() {
    m_ballCamera = new Limelight("limelight-ball");
    m_ballCamera.setPipeline(3);
  }

  public void setBallPipeline() {
    m_ballCamera.setPipeline(3);
  }
  /**
   * Is there a ball (boolean)
   * Decides whether there is a ball based on area
   * Note: Area is represented as percentage of limelight screen
   * Returns boolean true if a ball is recognized
   */
  public boolean IsBall() {
    double ballArea = m_ballCamera.getTargetArea();
    boolean Ball;
    if (ballArea >= 0.1) {
      Ball = true;
    } else {
      Ball = false;
    }
    return Ball;
  }

  /**
   * finds the x-angle/rotation of a ball
   * 
   * @return rotation angle
   */
  public double ballAngle() {
    double Angle = m_ballCamera.getHorizontalTargetOffsetAngle();
    return Angle;
  }

  /**
   * Allows user input to set pipeline
   * 
   * @param color (boolean)
   *              boolean 'color' true sets to pipeline 0 (red?)
   *              boolean 'color' false sets to pipeline 1 (blue?)
   */
  public void ballPipeline(boolean color) {
    if (color == true) {
      m_ballCamera.setPipeline(0);
    } else {
      m_ballCamera.setPipeline(1);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
