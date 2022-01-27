// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// libraries needed for NavX
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

public class Gyro extends SubsystemBase {
  // subsystem shuffleboard controls
  private NetworkTableEntry m_gyroPitch;
  private NetworkTableEntry m_gyroYaw;
  // make our gyro object
  AHRS gyro;

  /** Creates a new Gyro. */
  public Gyro() {
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
    initializeShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShuffleboard();
  }

  /**
   * Gets the yaw of the robot
   * 
   * @return current yaw value (-180 to 180)
   */
  public double getYaw() {
    return gyro.getYaw();

  }

  /**
   * Gets the pitch of the robot
   * 
   * @return current pitch value (-180 to 180)
   */
  public double getPitch() {
    return gyro.getPitch();
  }

  /**
   * Resets yaw to zero
   */
  public void resetGyro() {
    gyro.reset();
  }

  /**
   * Accumulated yaw
   * 
   * @return
   */
  public double continuousYaw() {
    return gyro.getAngle();
  }

  /** Gyro Shuffleboard */

  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Gyroscope");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("Gyroscope", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    m_gyroPitch = l1.add("Gyro(deg)", 0.0).getEntry();
    m_gyroYaw = l1.add("Gyro(deg)", 0.0).getEntry();
  }

  /** Update subsystem shuffle board page with current Gyro values */
  private void updateShuffleboard() {
    // write current robot Gyro
    m_gyroPitch.setDouble(getYaw());
    m_gyroYaw.setDouble(getYaw());
  }

}