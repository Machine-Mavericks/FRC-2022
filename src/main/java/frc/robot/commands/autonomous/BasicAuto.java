// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.TrajectoryConstants;
import frc.robot.commands.AutoDriveToPose;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.FollowPath;
import frc.robot.commands.SteerTowardsBall;
import frc.robot.commands.SteerTowardsHub;

/**
 * Basic auto which starts on right tarmac, grabs nearest ball, and shoots both
 */
public class BasicAuto extends SequentialCommandGroup {
  /** Creates a new BasicAuto. */
  public BasicAuto() {
    addCommands(
      new InstantCommand(()-> RobotContainer.m_shooter.setShooterSpeed(RobotContainer.m_shooter.ChosenIdleSpeed.getDouble(3000))),
      // Intake the ball
      new SteerTowardsBall(true, 2),
      // Turn left until hub is in view
      new TurnToHubCommand(0.5, 1),
      //get within shooting range
      new AutoHubDistanceCommand(3, 0.2),
      // Shoot first ball
      new AutoShootCommand(AutoShootCommand.HIGH_SPEED).deadlineWith(new SteerTowardsHub()),
      // Shoot second ball
      new AutoShootCommand(AutoShootCommand.HIGH_SPEED).deadlineWith(new SteerTowardsHub()),
      // Make a dash for ball near station
      new AutoDriveToPose(new Pose2d(new Translation2d(2.5, 2.5), Rotation2d.fromDegrees(-135)), 1, 0.10, 3),
      new SteerTowardsBall(true, 1),
      new DelayCommand(0.25),
      // Drive back to shooting position
      new AutoDriveToPose(new Pose2d(new Translation2d(6.5, 2.5), Rotation2d.fromDegrees(-135)), 0.5, 0.10, 3),
      // Turn left until hub is in view
      new TurnToHubCommand(0.5, 1),
      //get within shooting range
      new AutoHubDistanceCommand(3, 0.2),
      // Shoot first ball
      new AutoShootCommand(AutoShootCommand.HIGH_SPEED).deadlineWith(new SteerTowardsHub())
    );
  }
}
