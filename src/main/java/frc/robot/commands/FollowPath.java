// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotMap;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveOdometry;

public class FollowPath extends CommandBase {

  private Drivetrain m_drivetrain;
  private SwerveOdometry m_odometry;

  public final static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
      Drivetrain.TRACKWIDTH_METERS);

  /** Creates a new FollowPath. */
  public FollowPath(Drivetrain m_drivetrain) {
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            RobotMap.ODOMETRY.ksVolts,
            RobotMap.ODOMETRY.kvVoltSecondsPerMeter,
            RobotMap.ODOMETRY.kaVoltSecondsSquaredPerMeter),
        kDriveKinematics,
        10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        RobotMap.ODOMETRY.kMaxSpeedMetersPerSecond,
        RobotMap.ODOMETRY.kMaxAccelerationMetersPerSecondSquared)

            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kDriveKinematics)
            // Apply volatge constraint
            .addConstraint(autoVoltageConstraint);

    // Create example Trajectory
    Trajectory autoTrajectory = TrajectoryGenerator.generateTrajectory(
        // Define starting point
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through a list of defined points
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)),
        // Define Endpoint
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(
        autoTrajectory,
        m_odometry::getPose2d,
        new RamseteController(RobotMap.AUTONOMOUS.kRamseteB, RobotMap.AUTONOMOUS.kRamseteZeta),
        new SimpleMotorFeedforward(
            RobotMap.Drivetrain.ksVolts,
            RobotMap.Drivetrain.kvVoltSecondsPerMeter,
            RobotMap.Drivetrain.kaVoltSecondsSquaredPerMeter),
        kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(RobotMap.ODOMETRY.kPDriveVel, 0, 0),
        new PIDController(RobotMap.ODOMETRY.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::driveVoltageOutput,
        m_drivetrain);

    // Reset odometry to the starting pose of the trajectory
    m_odometry.resetOdometry(autoTrajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}