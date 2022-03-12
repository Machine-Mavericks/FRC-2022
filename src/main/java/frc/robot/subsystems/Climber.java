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
  NetworkTableEntry leftForwardLimit;
  NetworkTableEntry rightReverseLimit;
  NetworkTableEntry leftReverseLimit;
  NetworkTableEntry rightForwardLimit;
  NetworkTableEntry leftMotorPosition;
  NetworkTableEntry rightMotorPosition;

  private TalonFX m_leftClimberFalcon; 
  private TalonFX m_rightClimberFalcon; 
 

  public Climber() {
    
    // create falcon motors
    m_leftClimberFalcon= new TalonFX(RobotMap.CANID.LEFT_CLIMBER_FALCON);
    m_rightClimberFalcon= new TalonFX(RobotMap.CANID.RIGHT_CLIMBER_FALCON);
  
    // set factory default settings
    m_leftClimberFalcon.configFactoryDefault();
    m_rightClimberFalcon.configFactoryDefault();

    // select quadrature encoder (first parameter) as primary feedback sensor (second parameter=0)
    // Third parameter kTimeoutMs is timeout to wait for Talon to confirm update - set to 0 for no checking
    //m_leftClimberFalcon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,0);
    //m_rightClimberFalcon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,0);
    
    // set nominal and peak outputs of drive 
    // nominal = 0, peak is +1 or -1 depending on direction
    // second parameter is timeout (use 0)
    // m_leftClimberFalcon.configNominalOutputForward(0, 0);
    // m_leftClimberFalcon.configNominalOutputReverse(0, 0);
    // m_leftClimberFalcon.configPeakOutputForward (0.25, 0);      // % peak voltage
    // m_leftClimberFalcon.configPeakOutputReverse (-0.25, 0);     // % peak voltage
    // m_rightClimberFalcon.configNominalOutputForward(0, 0);
    // m_rightClimberFalcon.configNominalOutputReverse(0, 0);
    // m_rightClimberFalcon.configPeakOutputForward (0.25, 0);      // % peak voltage
    // m_rightClimberFalcon.configPeakOutputReverse (-0.25, 0);     // % peak voltage

    // set up forward direction soft (software) limit - set to maximum allowed encoder pulse counts at elevator top
    m_leftClimberFalcon.configForwardSoftLimitEnable(true,0);
    m_leftClimberFalcon.configForwardSoftLimitThreshold(8.80*48*2048.0);
    m_rightClimberFalcon.configForwardSoftLimitEnable(true,0);
    m_rightClimberFalcon.configForwardSoftLimitThreshold(8.80*48*2048.0);

    // set up reverse direction soft limit - assumes encoder pulse at bottom is 0
    m_leftClimberFalcon.configReverseSoftLimitEnable(true, 0);
    m_leftClimberFalcon.configReverseSoftLimitThreshold(0);
    m_rightClimberFalcon.configReverseSoftLimitEnable(true, 0);
    m_rightClimberFalcon.configReverseSoftLimitThreshold(0);

    // configure motor drive limit switches
    m_leftClimberFalcon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    m_rightClimberFalcon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    // m_leftClimberFalcon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);
    // m_rightClimberFalcon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 0);

    // right motor spins in opposite direction
    m_leftClimberFalcon.setInverted(InvertType.InvertMotorOutput);
    //m_rightClimberFalcon.setInverted(InvertType.InvertMotorOutput);
  
    // set allowable closed loop error
    //m_leftClimberFalcon.configAllowableClosedloopError(0, 4096.0);
    //m_leftClimberFalcon.configAllowableClosedloopError(0, 4096.0);
    
    // set PID gains
    m_leftClimberFalcon.config_kP(0, 0.25, 0);
    m_leftClimberFalcon.config_kI(0, 0.0, 0);
    m_leftClimberFalcon.config_kD(0, 0.0, 0);
    m_rightClimberFalcon.config_kP(0, 0.25, 0);
    m_rightClimberFalcon.config_kI(0, 0.0, 0);
    m_rightClimberFalcon.config_kD(0, 0.0, 0);

    // reset encoder positions
    m_leftClimberFalcon.setSelectedSensorPosition(0, 0, 0);
    m_rightClimberFalcon.setSelectedSensorPosition(0, 0, 0);

    // initialize shuffleboard
    initializeShuffleboard();
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
  }


  /** set motor posiiton */
  public void setRightMotorPosition(int pos)
  {  m_rightClimberFalcon.set(ControlMode.Position, pos); }
  public void setLeftMotorPosition(int pos)
  {  m_leftClimberFalcon.set(ControlMode.Position, pos); }
  public int getRightMotorPosition()
  {  return (int)m_rightClimberFalcon.getClosedLoopTarget(0); }
  public int getLeftMotorPosition()
  {  return (int)m_leftClimberFalcon.getClosedLoopTarget(0); }



  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
   // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("climber");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("climber", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    leftForwardLimit = l1.add("left fwd limit", 0.0).getEntry();
    rightReverseLimit = l1.add("right rev limit", 0.0).getEntry();
    rightForwardLimit = l1.add("right fwd limit", 0.0).getEntry();
    leftReverseLimit = l1.add("left rev limit", 0.0).getEntry();
  
    ShuffleboardLayout l2 = Tab.getLayout("climber", BuiltInLayouts.kList);
    l2.withPosition(2, 0);
    l2.withSize(1, 2);
    leftMotorPosition = l1.add("left mtr pos", 0.0).getEntry();
    rightMotorPosition = l1.add("right mtr pos", 0.0).getEntry();
  }

  /** Update subsystem shuffle board page with climber values */
  private void updateShuffleboard() {
    // write limit switch statuses
    leftForwardLimit.setDouble(m_leftClimberFalcon.getSensorCollection().isFwdLimitSwitchClosed());
    rightReverseLimit.setDouble(m_rightClimberFalcon.getSensorCollection().isRevLimitSwitchClosed());
    rightForwardLimit.setDouble(m_rightClimberFalcon.getSensorCollection().isFwdLimitSwitchClosed());
    leftReverseLimit.setDouble(m_leftClimberFalcon.getSensorCollection().isRevLimitSwitchClosed());
  
    leftMotorPosition.setDouble(m_leftClimberFalcon.getSelectedSensorPosition(0));
    rightMotorPosition.setDouble(m_rightClimberFalcon.getSelectedSensorPosition(0));
  }



}
