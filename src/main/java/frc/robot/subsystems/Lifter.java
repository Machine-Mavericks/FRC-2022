// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Lifter extends SubsystemBase {
  //two talon
  VictorSPX leaderLifterTalon = new VictorSPX(RobotMap.CANID.L_LIFTER_TALON); 
  VictorSPX followerLifterTalon = new VictorSPX(RobotMap.CANID.R_LIFTER_TALON);

  public NetworkTableEntry lifterSpeedEntry;

  /** Creates a new Lifter. */
  public Lifter() {

    followerLifterTalon.follow(leaderLifterTalon);
    followerLifterTalon.setInverted(InvertType.OpposeMaster);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void liftBalls(){
    leaderLifterTalon.set(ControlMode.PercentOutput, -0.5); //TODO: change this to be set in the motors
  }

  public void stopMotor() {
    leaderLifterTalon.set(ControlMode.PercentOutput, 0.0);
  }
}
