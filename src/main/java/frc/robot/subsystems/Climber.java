// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
 
  // shuffleboard controls
  NetworkTableEntry forwardLimit;
  NetworkTableEntry reverseLimit;
  NetworkTableEntry motorPosition;

  private TalonFX m_climberFalcon; 

  public Climber() {
    
    // create falcon motors
    m_climberFalcon= new TalonFX(RobotMap.CANID.CLIMBER_FALCON);
  
    // set factory default settings
    m_climberFalcon.configFactoryDefault();

    // select quadrature encoder (first parameter) as primary feedback sensor (second parameter=0)
    // Third parameter kTimeoutMs is timeout to wait for Talon to confirm update - set to 0 for no checking
    //m_climberFalcon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,0);
    //m_rightClimberFalcon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,0);
    
    // set nominal and peak outputs of drive 
    // nominal = 0, peak is +1 or -1 depending on direction
    // second parameter is timeout (use 0)
    // m_climberFalcon.configNominalOutputForward(0, 0);
    // m_climberFalcon.configNominalOutputReverse(0, 0);
    // m_climberFalcon.configPeakOutputForward (0.25, 0);      // % peak voltage
    // m_climberFalcon.configPeakOutputReverse (-0.25, 0);     // % peak voltage
    // m_rightClimberFalcon.configNominalOutputForward(0, 0);
    // m_rightClimberFalcon.configNominalOutputReverse(0, 0);
    // m_rightClimberFalcon.configPeakOutputForward (0.25, 0);      // % peak voltage
    // m_rightClimberFalcon.configPeakOutputReverse (-0.25, 0);     // % peak voltage

    // set up forward direction soft (software) limit - set to maximum allowed encoder pulse counts at elevator top
    m_climberFalcon.configForwardSoftLimitEnable(true,0);
    m_climberFalcon.configForwardSoftLimitThreshold(8.80*48*2048.0);

    // set up reverse direction soft limit - assumes encoder pulse at bottom is 0
    m_climberFalcon.configReverseSoftLimitEnable(true, 0);
    m_climberFalcon.configReverseSoftLimitThreshold(0);

    // configure motor drive limit switches
    m_climberFalcon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);

    // right motor spins in opposite direction
    m_climberFalcon.setInverted(InvertType.InvertMotorOutput);
    //m_rightClimberFalcon.setInverted(InvertType.InvertMotorOutput);
  
    // set allowable closed loop error
    //m_climberFalcon.configAllowableClosedloopError(0, 4096.0);
    //m_climberFalcon.configAllowableClosedloopError(0, 4096.0);
    
    // set PID gains
    m_climberFalcon.config_kP(0, 0.25, 0);
    m_climberFalcon.config_kI(0, 0.0, 0);
    m_climberFalcon.config_kD(0, 0.0, 0);

    // reset encoder positions
    m_climberFalcon.setSelectedSensorPosition(0, 0, 0);

    // initialize shuffleboard
    initializeShuffleboard();
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
  }


  /** set motor posiiton */
  public void setMotorPosition(int pos)
  {  m_climberFalcon.set(ControlMode.Position, pos); }
  public int getMotorPosition()
  {  return (int)m_climberFalcon.getClosedLoopTarget(0); }

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
   // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("climber");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("climber", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    forwardLimit = l1.add(" fwd limit", 0.0).getEntry();
    reverseLimit = l1.add(" rev limit", 0.0).getEntry();
  
    ShuffleboardLayout l2 = Tab.getLayout("climber", BuiltInLayouts.kList);
    l2.withPosition(2, 0);
    l2.withSize(1, 2);
    motorPosition = l1.add(" mtr pos", 0.0).getEntry();
  }

  /** Update subsystem shuffle board page with climber values */
  private void updateShuffleboard() {
    // write limit switch statuses
    forwardLimit.setDouble(m_climberFalcon.getSensorCollection().isFwdLimitSwitchClosed());
    reverseLimit.setDouble(m_climberFalcon.getSensorCollection().isRevLimitSwitchClosed());
  
    motorPosition.setDouble(m_climberFalcon.getSelectedSensorPosition(0));
  }



}
