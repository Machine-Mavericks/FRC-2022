// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
   * Decides if a target is present an in range
   * Note: Range can (and will) be adjusted post testing
   * 
   * @return boolean (true if target, false if not)
   */
  public boolean IsTarget() {
    boolean target = m_hubCamera.isTargetPresent();
    double distance = EstimateDistance();
    boolean yesTarget;
    
    // based on Feb 22/2022 test data. If distance between 7 and 11
    if (target == true && distance >=1.98 && distance <=3.35) {
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
    
    // based on testing data from Feb 22/2022 (x3.208 to convert ft to m)
    double distance = 0.3048 *(0.0125 * ty * ty - 0.4959 * ty + 10.158);
    
    // return distance estimate (m)
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
  // TODO - to be removed
  public double getHubAngle(){
    return RotationAngle();
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

  /**
   * Finds the RPM (rates per minute) needed to shoot a distance
   * Note: equation will change with finished shooter + testing
   * 
   * @return RPM as double
   */
  public double DistanceRPM() {
    // get distance in ft
    double ft = EstimateDistance()*.3048;
    
    // speed from test data Feb 22/2022
    double RPM = 12.456 * ft * ft - 120.39 * ft + 3319.2;
    
    // fudge factors - best fit curve not completely matching data
    if (ft>9.0 && ft<10.0)
      RPM += 25.0;
    if (ft>7.25 && ft<7.75)
      RPM -= 15.0;
    if (ft>10.25)
     RPM += (ft-10.25)*150;
     

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
    l1.withPosition(2, 0);
    l1.withSize(2, 4);
    m_ty = l1.add("limelight ty", 0.0).getEntry();
    m_distance = l1.add("Estimated Target Distance", 0.0).getEntry();
    m_RPM = l1.add("Estimated Target RPM", 0.0).getEntry();
    
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
    m_RPM.setDouble(DistanceRPM());
  }

}
