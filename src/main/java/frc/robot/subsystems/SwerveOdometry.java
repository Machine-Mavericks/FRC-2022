// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Swerve odometry is used to estimate current robot x, y position and angle.
// x and y coordinates are relative to when odometry was last reset

// Jan 25/2022
// TODO  a) need gyro module, and to connect up odometry to gyro

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class SwerveOdometry extends SubsystemBase {

  // constant to convert degrees to radians
  final float DEGtoRAD = (float) (3.1415926 / 180.0);

  // create swerve drive odometry object
  private SwerveDriveOdometry m_Odometry;

  // subsystem shuffleboard controls
  private NetworkTableEntry RobotX;
  private NetworkTableEntry RobotY;
  private NetworkTableEntry RobotAngle;
  private NetworkTableEntry GyroAngle;

  private NetworkTableEntry InitialX;
  private NetworkTableEntry InitialY;
  private NetworkTableEntry InitialAngle;

  /** Creates a new SwerveOdometry. */
  public SwerveOdometry() {

    // create robot odometry - set to (0,0,0)(x,y,ang)

    // initialize swerve drive odometry
    m_Odometry = new SwerveDriveOdometry(RobotContainer.m_drivetrain.getKinematics(),
        new Rotation2d(0.0),
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)));

    // create odometry shuffleboard page
    initializeShuffleboard();
  }

  // -------------------- Initialize and Update Odometry Methods

  /**
   * Initialize robot odometry use shuffleboard settings and current gyro angle
   */
  public void InitializefromShuffleboard() {
    setPosition(InitialX.getDouble(0.0),
        InitialY.getDouble(0.0),
        InitialAngle.getDouble(0.0),
        0.0);
  } // get gyro angle from subsystem

  /** Initialize robot odometry to zero */
  public void InitializetoZero() {
    setPosition(0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Used to set or reset odometry to fixed position
   * x, y displacement in m, robot angle in deg, gyro in deg
   */
  public void setPosition(double x, double y, double robotangle, double gyroangle) {

    // make robot position vector
    Pose2d position = new Pose2d(x, y, new Rotation2d(robotangle * DEGtoRAD));

    // set robot odometry
    m_Odometry.resetPosition(position, new Rotation2d(gyroangle * DEGtoRAD));
  }

  /** Update current robot dometry - called by scheduler at 50Hz */
  @Override
  public void periodic() {

    // get gyro angle (in degrees) and make rotation vector
    Rotation2d gyroangle = new Rotation2d(0.0 * DEGtoRAD); // TODO: get current angle from gyro subystem

    // get states of all swerve modules from subsystem
    SwerveModuleState[] states = RobotContainer.m_drivetrain.getSwerveStates();
    
    // ensure we have proper length array of states before accessing elements of array
    if (states.length >=4) {
      // update the robot's odometry
      m_Odometry.update(gyroangle, states[0], states[1], states[2], states[3]);
    }
    
    // update odemetry shuffleboard page
    updateShuffleboard();
  }

  // -------------------- Robot Current Odometry Access Methods --------------------

  /** return robot's current position vector Pose2d */
  public Pose2d getPose2d() {
    return m_Odometry.getPoseMeters();
  }

  /** Return current odometry x displacement (in m) */
  public double getX() {
    return m_Odometry.getPoseMeters().getX();
  }

  /** Return current odometry y displacement (in m) */
  public double getY() {
    return m_Odometry.getPoseMeters().getY();
  }

  // return current odometry angle (in deg)
  public double getAngle() {
    return m_Odometry.getPoseMeters().getRotation().getDegrees();
  }


  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Odometry");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("Odometry", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 4);
    RobotX = l1.add("X (m)", 0.0).getEntry();
    RobotY = l1.add("Y (m)", 0.0).getEntry();
    RobotAngle = l1.add("Angle(deg)", 0.0).getEntry();
    GyroAngle = l1.add("Gyro(deg)", 0.0).getEntry();

    // Controls to set initial robot position and angle
    // TODO Unsure if this will be needed. If not, can be deleted.
    ShuffleboardLayout l2 = Tab.getLayout("Initial Position", BuiltInLayouts.kList);
    l2.withPosition(4, 0);
    l2.withSize(1, 3);
    InitialX = l2.add("X (m)", 0.0).getEntry();           // eventually can use .addPersistent once code finalized
    InitialY = l2.add("Y (m)", 0.0).getEntry();           // eventually can use .addPersentent once code finalized
    InitialAngle = l2.add("Angle(deg)", 0.0).getEntry();  // eventually can use .addPersentent once code finalized
  }

  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {
    // write current robot odometry
    RobotX.setDouble(getX());
    RobotY.setDouble(getY());
    RobotAngle.setDouble(getAngle());
    GyroAngle.setDouble(getAngle());
  }

} // end SwerveOdometry Class
