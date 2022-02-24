// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubTargeting extends SubsystemBase {
  private Limelight m_hubCamera;
  private NetworkTableEntry ty;
  private NetworkTableEntry distance;
  private NetworkTableEntry RPM;
  private NetworkTableEntry ready;

  /** Creates a new HubTargeting. */
  public HubTargeting() {
    m_hubCamera = new Limelight("limelight-hub");
    m_hubCamera.setPipeline(0);
    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Range (m)", EstimateDistance());
    
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
    double distance = 0.0065655 * ty * ty - 0.2198 * ty +3.73592;
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

  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Hub Target");

    // create controls to display limelight ty, target ready, estimated distance and
    // estimated RPM
    ShuffleboardLayout l1 = Tab.getLayout("Hub Target", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    ty = l1.add("limelight ty", 0.0).getEntry();
    distance = l1.add("Estimated Distance of Target", 0.0).getEntry();
    RPM = l1.add("Estimated RPM needed for Target", 0.0).getEntry();
    ready = l1.add("Ready to Shoot", 0.0).getEntry();
  }

  /** Update subsystem shuffle board page with current targeting values */
  private void updateShuffleboard() {
    // write current robot Gyro
    ty.setDouble(m_hubCamera.getVerticalTargetOffsetAngle());
    distance.setDouble(EstimateDistance());
    RPM.setDouble(DistanceRPM());
    ready.setBoolean(ReadyToShoot());
  }

}
