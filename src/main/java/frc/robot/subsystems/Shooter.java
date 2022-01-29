// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.RobotContainer;


public class Shooter extends SubsystemBase {

  public enum HoodState {
    HIGH, LOW
  }

  private TalonFX rightShooterFalcon = new TalonFX(RobotMap.CANID.RIGHT_SHOOTER_FALCON);
  private TalonFX leftShooterFalcon = new TalonFX(RobotMap.CANID.LEFT_SHOOTER_FALCON);
  private DoubleSolenoid shooterSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
      RobotMap.SHOOTER_SOLENOID_EXTEND, RobotMap.SHOOTER_SOLENOID_RETRACT);

  private static final DoubleSolenoid.Value PISTON_EXTENDED = DoubleSolenoid.Value.kForward;
  private static final DoubleSolenoid.Value PISTON_RETRACTED = DoubleSolenoid.Value.kReverse;
  private static double MOTORSPEED = 0.8;
  private double idleSpeed;
  private boolean isIdling;
  /** Creates a new Shooter. */
  public Shooter() {
    leftShooterFalcon.follow(rightShooterFalcon);
    leftShooterFalcon.setInverted(InvertType.OpposeMaster);
    shooterSolenoid.set(PISTON_RETRACTED);
    rightShooterFalcon.set(ControlMode.PercentOutput, 0);
    isIdling = false;
    rightShooterFalcon.configPeakOutputForward(1,0);
    rightShooterFalcon.configPeakOutputReverse(0,0);
    double idleSpeed = 0.1;
    //rightShooterFalcon.configOpenLoopRamp(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * This method will set the idle speed of the shooter
   * @param newIdleSpeed the desired idle speed 
   */

  public void setIdleSpeed(double newIdleSpeed) {
    idleSpeed = newIdleSpeed;
    if (isIdling) {
      idle();
    }
  }
  /**
   * This method will set the shooter's motors to the given idle speed
   */
  public void idle() {
    rightShooterFalcon.set(ControlMode.PercentOutput, idleSpeed);
    isIdling = true;
  }
  /**
   * This method will set the motors to the given motor speed
   * @param shooterSpeed the desired motor speed between 1 and -1
   */
  public void setShooterSpeed(double shooterSpeed) {
    rightShooterFalcon.set(ControlMode.PercentOutput, shooterSpeed);
    isIdling = false;
  }
  /**
   * This method will raise or lower the hood on the shooter for high or low shooting
   * @param state either HIGH or LOW, the desired state for the hood of the shooter to be in
   */
  public void setShooterHood(HoodState state) {
    switch(state) {
      case HIGH:
        shooterSolenoid.set(PISTON_EXTENDED);
        break;
      case LOW:
        shooterSolenoid.set(PISTON_RETRACTED);
        break;
    }
  }
}
