// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  private TalonFX intakeFalcon = new TalonFX(RobotMap.CANID.INTAKE_FALCON);
  
  public static double MOTORSPEED = 0.8;

  /** Creates a new Intake. */
  public Intake() {
    intakeFalcon.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
/**
 * This function will set the falcon motor speed to whatever double is passed in
 * @param motorSpeed percentage motor power, ranging from 0.0 to 1.0
 */
  public void setMotorSpeed(double motorSpeed) {
    intakeFalcon.set(ControlMode.PercentOutput, motorSpeed);
  }
}
