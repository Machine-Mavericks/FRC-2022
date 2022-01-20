// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// libraries needed for NavX
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  // make our gyro object
  AHRS gyro;

  /** Creates a new Gyro. */
  public Gyro() {
    gyro = new AHRS(SPI.Port.kMXP);
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Gets the yaw of the robot
   * @return current yaw value (-180 to 180)
   */
  public float getYaw() {
    return gyro.getYaw();

  }

  /**
   * Gets the pitch of the robot
   * @return current pitch value (-180 to 180)
   */
  public float getPitch() {
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
   * @return 
   */
  public double continuousYaw() {
    return gyro.getAngle();
  }
}
