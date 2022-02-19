// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubTargeting extends SubsystemBase {
  public Limelight m_hubCamera;

  /** Creates a new HubTargeting. */
  public HubTargeting() {
    m_hubCamera = new Limelight("limelight-hub");
    m_hubCamera.setPipeline(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Decides if a target is present an in range
   * Note: Range can (and will) be adjusted post testing
   * 
   * @return boolean (true if target, false if not)
   */
  public boolean IsTarget() {
    boolean target = m_hubCamera.isTargetPresent();
    double distance = EstimateDistance();
    boolean yesTarget;
    if (target == true && 1.6 <= distance && 7 >= distance) {
      yesTarget = true;
    } else {
      yesTarget = false;
    }
    return yesTarget;
  }

  /**
   * Estimates the distance of the hub
   * Note: Equation will change with mounted limelight and testing
   * 
   * @return distance in meters
   */
  public double EstimateDistance() {
    double ty = m_hubCamera.getVerticalTargetOffsetAngle();
    double distance = 0.0931 * ty * ty - 0.5042 * ty - 9.0222;
    return distance;
  }

  /**
   * finds angle of rotation to hub
   * 
   * @return rotation angle
   */
  public double RotationAngle() {
    double tx = m_hubCamera.getHorizontalTargetOffsetAngle();
    return tx;
  }

  /**
   * Finds if targeting is ready
   * Note: angle displacement parameters are placement holders
   * 
   * @return boolean
   */
  public boolean ReadyToShoot() {
    double targetAngle = RotationAngle();
    Boolean target = IsTarget();
    Boolean ready;
    if (targetAngle <= 5 && targetAngle >= -5 && target == true) {
      ready = true;
    } else {
      ready = false;
    }
    return ready;
  }

  public double getHubAngle(){
    return RotationAngle();
  }

  /**
   * Finds the RPM (rates per minute) needed to shoot a distance
   * Note: equation will change with finished shooter + testing
   * 
   * @return RPM as double
   */
  public double DistanceRPM() {
    double met = EstimateDistance();
    double RPM = 7.5 * met * met - 52.5 * met + 120;
    return RPM;
  }

}
