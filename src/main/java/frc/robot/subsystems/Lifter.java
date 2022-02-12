// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Lifter extends SubsystemBase {
  //two talon
  TalonSRX leaderLifterTalon = new TalonSRX(RobotMap.CANID.L_LIFTER_TALON); 
  TalonSRX followerLifterTalon = new TalonSRX(RobotMap.CANID.R_LIFTER_TALON);

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
    leaderLifterTalon.set(ControlMode.PercentOutput, lifterSpeedEntry.getDouble(1.0));
  }

  public void initializeShuffleboard(){
    ShuffleboardTab Tab = Shuffleboard.getTab("Lifter");
    lifterSpeedEntry = Shuffleboard.getTab("Lifter")
        .add("Lifter Speed", 1.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();
  }

}
