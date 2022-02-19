// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Lifter extends SubsystemBase {

  public NetworkTableEntry limitSwitch;
  public NetworkTableEntry intakeMotorSpeed;
  //two talon
  VictorSPX leaderLifterTalon = new VictorSPX(RobotMap.CANID.L_LIFTER_TALON); 
  VictorSPX followerLifterTalon = new VictorSPX(RobotMap.CANID.R_LIFTER_TALON);

  public NetworkTableEntry lifterSpeedEntry;

  public DigitalInput liftLimit = new DigitalInput(RobotMap.LIFTER_LIMIT_ID);


  /** Creates a new Lifter. */
  public Lifter() {

    initializeShuffleboard();
    followerLifterTalon.follow(leaderLifterTalon);
    followerLifterTalon.setInverted(InvertType.OpposeMaster);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
    
  }

  public void liftBalls(){
    leaderLifterTalon.set(ControlMode.PercentOutput, RobotMap.BALL_LIFTER_SPEED); 
  }

  public void releaseBalls(){
    leaderLifterTalon.set(ControlMode.PercentOutput, -RobotMap.BALL_LIFTER_SPEED); 
  }

  public void stopMotor() {
    leaderLifterTalon.set(ControlMode.PercentOutput, 0.0);
    // leaderLifterTalon.NeutralMode.Brake=(2);

  }

  public void initializeShuffleboard() {
    ShuffleboardTab Tab = Shuffleboard.getTab("Lifter");
    ShuffleboardLayout l1 = Tab.getLayout("Shooter", BuiltInLayouts.kList);
    l1.withPosition(3, 0);
    l1.withSize(1, 4);
    limitSwitch = l1.add("limswitch", 0.0).getEntry();
  }
  public void updateShuffleboard() {

    limitSwitch.setBoolean(liftLimit.get());

  }
}
