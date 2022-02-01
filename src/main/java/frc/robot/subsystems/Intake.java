// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


public class Intake extends SubsystemBase {
  private NetworkTableEntry m_speed;
  private NetworkTableEntry m_speedslider;

  private TalonFX intakeFalcon = new TalonFX(RobotMap.CANID.INTAKE_FALCON);
  
  public static double MOTORSPEED = 0.8;

  /** Creates a new Intake. */
  public Intake() {
    intakeFalcon.set(ControlMode.PercentOutput, 0);
    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
  }
/**
 * This function will set the falcon motor speed to whatever double is passed in
 * @param motorSpeed percentage motor power, ranging from 0.0 to 1.0
 */
  public void setMotorSpeed(double motorSpeed) {
    intakeFalcon.set(ControlMode.PercentOutput, motorSpeed);
  }

  public double getMotorSpeed() {
    return 0;
  }

    // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create Intake page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Intake");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("Intake", BuiltInLayouts.kList);
    
    l1.withPosition(0, 0);
    l1.withSize(1, 4);

    m_speed = l1.add("Speed", 0.0).getEntry();
    m_speedslider = Tab.add("Speed", MOTORSPEED).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1)).getEntry();
    // m_robotX = l1.add("X (m)", 0.0).getEntry();
    // m_robotY = l1.add("Y (m)", 0.0).getEntry();
    // m_robotAngle = l1.add("Angle(deg)", 0.0).getEntry();
    // m_gyroAngle = l1.add("Gyro(deg)", 0.0).getEntry();
  }

  /** Update subsystem shuffle board page with current Intake values */
  private void updateShuffleboard() {
    // write current intake data
    m_speed.setDouble(getMotorSpeed());
    m_speedslider.getDouble(0.0);
  }
}
