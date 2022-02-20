// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

  public enum HoodState {
    HIGH, LOW
  }

  private TalonFX rightShooterFalcon = new TalonFX(RobotMap.CANID.RIGHT_SHOOTER_FALCON);
  private TalonFX leftShooterFalcon = new TalonFX(RobotMap.CANID.LEFT_SHOOTER_FALCON);

  public NetworkTableEntry ChosenSpeed;
  public NetworkTableEntry ChosenIdleSpeed;
  private NetworkTableEntry motorSpeed;
  private NetworkTableEntry motorVoltage;
  private NetworkTableEntry rightMotorCurrent;
  private NetworkTableEntry leftMotorCurrent;
  private NetworkTableEntry targetSpeed;

  /** Creates a new Shooter. */
  public Shooter() {
    rightShooterFalcon.setInverted(TalonFXInvertType.Clockwise);
    leftShooterFalcon.follow(rightShooterFalcon);
    leftShooterFalcon.setInverted(InvertType.OpposeMaster);

    rightShooterFalcon.configVoltageCompSaturation(12.0, 0);
    leftShooterFalcon.configVoltageCompSaturation(12.0, 0);

    rightShooterFalcon.config_kF(0,  0.047698, 0);
    rightShooterFalcon.config_kP(0, 0.35, 0);
    rightShooterFalcon.config_kI(0, 0.001, 0);

    // rightShooterFalcon.set(ControlMode.PercentOutput, 0);
    rightShooterFalcon.configPeakOutputForward(1, 0);
    rightShooterFalcon.configPeakOutputReverse(0, 0);
    initializeShuffleboard();
    // rightShooterFalcon.configOpenLoopRamp(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
  }


  /**
   * This method will set the motors to the given motor speed
   * 
   * @param shooterSpeed the desired motor speed in rpm
   */
  public void setShooterSpeed(double shooterSpeed) {
    rightShooterFalcon.set(ControlMode.Velocity,shooterSpeed* (2048/600.0));
  }
  /**
   * This method will raise or lower the hood on the shooter for high or low
   * shooting
   * 
   * @param state either HIGH or LOW, the desired state for the hood of the
   *              shooter to be in
   */
  // public void setShooterHood(HoodState state) {
  // switch(state) {
  // case HIGH:
  // shooterSolenoid.set(PISTON_EXTENDED);
  // break;
  // case LOW:
  // shooterSolenoid.set(PISTON_RETRACTED);
  // break;
  // }
  // }

  /** Shooter Shuffleboard */

  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */

  public void initializeShuffleboard() {
    ShuffleboardTab Tab = Shuffleboard.getTab("Shooter");
    ChosenIdleSpeed = Shuffleboard.getTab("Shooter")
        .add("Idle speed (RPM)", 1.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 5000))
        .getEntry();

    ChosenSpeed = Shuffleboard.getTab("Shooter")
        .add("shooter Speed (RPM)", 1.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 5000))
        .getEntry();

    // add RPM
    ShuffleboardLayout l1 = Tab.getLayout("Shooter", BuiltInLayouts.kList);
    l1.withPosition(3, 0);
    l1.withSize(1, 4);
    motorSpeed = l1.add("motor speed", 0.0).getEntry();
    motorVoltage = l1.add("motor voltage", 0.0).getEntry();
    leftMotorCurrent = l1.add("L motor current",0.0).getEntry();
    rightMotorCurrent = l1.add("R motor current",0.0).getEntry();
    targetSpeed = l1.add("Target Speed",0.0).getEntry();
  }

  public void updateShuffleboard() {

    motorSpeed.setDouble(rightShooterFalcon.getSelectedSensorVelocity() * ((10.0 / 2048.0) * 60));
    motorVoltage.setDouble(rightShooterFalcon.getMotorOutputVoltage());
    leftMotorCurrent.setDouble(leftShooterFalcon.getSupplyCurrent());
    rightMotorCurrent.setDouble(rightShooterFalcon.getSupplyCurrent());
    targetSpeed.setDouble(rightShooterFalcon.getClosedLoopTarget() * ((10.0 / 2048.0) * 60));
  }
}