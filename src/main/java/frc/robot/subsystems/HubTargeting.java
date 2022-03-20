// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HubTargeting extends SubsystemBase {
  private Limelight m_hubCamera;
  
  private NetworkTableEntry m_ty;
  private NetworkTableEntry m_distance;
  private NetworkTableEntry m_RPM;
  private NetworkTableEntry m_Hood;

  private NetworkTableEntry m_targetWithinRange;
  private NetworkTableEntry m_readyToShoot;
  private NetworkTableEntry m_targetDetected;
  

  /** Creates a new HubTargeting. */
  public HubTargeting() {
    m_hubCamera = new Limelight("limelight-hub");
    m_hubCamera.setPipeline(0);
    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    
    // update shuffleboard
    updateShuffleboard();
  
  }

  /**
   * Function to tell if target is visible in limelight.
   * @return boolean (true if target is in sight)
   */
  public boolean isTargetPresent(){
    return m_hubCamera.isTargetPresent();
  }

  /**
   * Decides if a target is present and in range
   * Note: Range can (and will) be adjusted post testing
   * 
   * @return boolean (true if target, false if not)
   */
  public boolean IsTarget() {
    boolean target = m_hubCamera.isTargetPresent();
    double distance = EstimateDistance();
    boolean yesTarget;
    
    // based on Feb 22/2022 test data. If distance between 7 and 11
    if (target == true && distance >=3.00) {
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
    double distance;
    // based on test data from March 19 2022
    //double distance = -0.001*ty*ty*ty + 0.0081*ty*ty - 0.1462*ty + 4.2368;
    //double distance = 0.0131*ty*ty - 0.1883*ty + 4.1267;
    if (ty<0.67)
       distance = 0.00008*ty*ty*ty*ty - 0.0003*ty*ty*ty + 0.0046*ty*ty - 0.1699*ty + 4.2458;
    else
      distance = 0.0039*ty*ty -0.1523*ty + 4.094;

    // return distance estimate (m)
    return distance;
  }

  /** finds angle of rotation to hub
   * @return rotation angle */
  public double getHubAngle(){
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
    double targetAngle = getHubAngle();
    Boolean target = IsTarget();
    Boolean ready;
    if (targetAngle <= 5 && targetAngle >= -5 && target == true) {
      ready = true;
    } else {
      ready = false;
    }
    return ready;
  }

  /** Finds the RPM (rates per minute) needed to shoot a distance
   * Note: equation will change with finished shooter + testing
   * @return RPM as double*/
  public double GetTargetRPM() {
    // get distance in m
    double m = EstimateDistance();
    
    // speed from test data March 19/2022
    double RPM = 380.02*m + 1842.2;

    return RPM;
  }

  /** Finds the Shooter Hood actuator Setting need to shoot a distance
   * @return hood actuator setting*/
  public double GetTargetHoodSetting() {
    // get distance in m
    double m = EstimateDistance();
    
    // speed from test data March 19/2022
    double hood = -0.0855*m*m +1.2416*m - 4.2243;

    return hood;
  }



  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Hub Target");

    // create controls to display limelight ty, target ready, estimated distance and
    // estimated RPM
    ShuffleboardLayout l1 = Tab.getLayout("Hub Target", BuiltInLayouts.kList);
    l1.withPosition(2, 0);
    l1.withSize(2, 4);
    m_ty = l1.add("limelight ty", 0.0).getEntry();
    m_distance = l1.add("Estimated Target Distance", 0.0).getEntry();
    m_RPM = l1.add("Estimated Target RPM", 0.0).getEntry();
    m_Hood = l1.add("Estimated Hood Setting", 0.0).getEntry();

    // does camera detect target
    m_targetDetected =Tab.add("Detected", false).withPosition(0,0).getEntry();
    m_targetWithinRange = Tab.add("Within Range", false).withPosition(0,1).getEntry();
    m_readyToShoot = Tab.add("Ready to Shoot", false).withPosition(0,2).getEntry();
  }

  /** Update subsystem shuffle board page with current targeting values */
  private void updateShuffleboard() {
    // target within range and ready to shoot
    m_targetDetected.setBoolean (m_hubCamera.isTargetPresent());
    m_targetWithinRange.setBoolean (IsTarget());
    m_readyToShoot.setBoolean(ReadyToShoot());
   
    // estimated target attributes
    m_ty.setDouble(m_hubCamera.getVerticalTargetOffsetAngle());
    m_distance.setDouble(EstimateDistance());
    m_RPM.setDouble(GetTargetRPM());
    m_Hood.setDouble(GetTargetHoodSetting());
  }

}
