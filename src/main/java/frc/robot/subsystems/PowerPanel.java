// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


/** Shows Robot Power Panel Data on Shuffleboard */
public class PowerPanel extends SubsystemBase {
  
  private PowerDistribution m_Panel;
  
  // subsystem shuffleboard controls
  private NetworkTableEntry m_Voltage;
  private NetworkTableEntry m_Current;
  private NetworkTableEntry m_Power;
  private NetworkTableEntry m_Energy;

  /** Creates a new PowerPanel. */
  public PowerPanel() {
    
    // create power distribution object
    m_Panel = new PowerDistribution(1, ModuleType.kRev);

    // set up shuffleboard page
    initializeShuffleboard();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // update the shuffleboard
    updateShuffleboard();
  }


  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Power Panel");

    // create controls PDP values
    ShuffleboardLayout l1 = Tab.getLayout("Gyroscope", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    m_Voltage = l1.add("Voltage(V)", 0.0).getEntry();
    m_Current = l1.add("Total Current(A)", 0.0).getEntry();
    m_Power = l1.add("Total Power(W)", 0.0).getEntry();
    m_Energy = l1.add("Total Energy(Wh)", 0.0).getEntry();
  }


  /** Update subsystem shuffle board page with current Gyro values */
  private void updateShuffleboard() {
    // write current robot Gyro
    m_Voltage.setDouble(m_Panel.getVoltage());
    m_Current.setDouble(m_Panel.getTotalCurrent());
    m_Power.setDouble(m_Panel.getTotalPower());
    m_Energy.setDouble(m_Panel.getTotalEnergy()*0.00027778);
  }


}
