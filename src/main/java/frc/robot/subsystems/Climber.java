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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import java.util.Map;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
 
  // shuffleboard controls
  NetworkTableEntry forwardLimit;
  NetworkTableEntry reverseLimit;
  NetworkTableEntry motorPosition;
  NetworkTableEntry encoderValue;

  private TalonFX m_climberFalcon; 

  public Climber() {
    // create falcon motors
    m_climberFalcon = new TalonFX(RobotMap.CANID.CLIMBER_FALCON);
  
    // set factory default settings
    m_climberFalcon.configFactoryDefault();

    // m_climberFalcon.configForwardSoftLimitEnable(true,0);
    // m_climberFalcon.configForwardSoftLimitThreshold(8.80*48*2048.0);

    // right motor spins in opposite direction
    m_climberFalcon.setInverted(InvertType.InvertMotorOutput);
      
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
  public void motorVelocity(double velocity)
  {  
    if (m_climberFalcon.getSelectedSensorPosition() < 0 || m_climberFalcon.getSelectedSensorPosition() >= 100) //TODO:
    {
      m_climberFalcon.set(ControlMode.Velocity, 0); 
    }else{
      m_climberFalcon.set(ControlMode.Velocity, velocity * 600 / 4096); 
    }
    
  }



  public NetworkTableEntry ClimbSpeed;
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

    ShuffleboardLayout l3 = Tab.getLayout("climber", BuiltInLayouts.kList);
    l3.withPosition(4, 0);
    l3.withSize(1, 2);
    encoderValue = l3.add("Climbing Motor Encoder Reading", 0.0).getEntry();

    ChosenAngle = Tab.add("Shooter angle", 1.0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -1.0, "max", 0.75))
    .getEntry();
  }

  /** Update subsystem shuffle board page with climber values */
  private void updateShuffleboard() {
    encoderValue.setDouble(m_climberFalcon.getSelectedSensorPosition(0));
  }
}
