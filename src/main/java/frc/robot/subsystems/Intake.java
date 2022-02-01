// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  private TalonFX leaderIntakeFalcon = new TalonFX(RobotMap.CANID.LEADER_INTAKE_FALCON);
  private TalonFX followerIntakeFalcon = new TalonFX(RobotMap.CANID.FOLLOWER_INTAKE_FALCON);
  private DoubleSolenoid pistonSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      RobotMap.PneumaticsChannel.INTAKE_SOLENOID_EXTEND, RobotMap.PneumaticsChannel.INTAKE_SOLENOID_RETRACT);

  private static final DoubleSolenoid.Value PISTON_EXTENDED = DoubleSolenoid.Value.kForward;
  private static final DoubleSolenoid.Value PISTON_RETRACTED = DoubleSolenoid.Value.kReverse;
  private static double MOTORSPEED = 0.8;

  /** Creates a new Intake. */
  public Intake() {
    followerIntakeFalcon.follow(leaderIntakeFalcon);
    followerIntakeFalcon.setInverted(InvertType.OpposeMaster);
    pistonSolenoid.set(PISTON_RETRACTED);
    leaderIntakeFalcon.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
/**
 * This function will extend the intake and rev up the intake falcons
 */
  public void extend() {
    pistonSolenoid.set(PISTON_EXTENDED);
    leaderIntakeFalcon.set(ControlMode.PercentOutput, MOTORSPEED);
  }
  /**
   * This function will retract the intake back in and turn off the intake falcons
   */
  public void retract() {
    pistonSolenoid.set(PISTON_RETRACTED);
    leaderIntakeFalcon.set(ControlMode.PercentOutput, 0);
  }
}
