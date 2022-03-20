// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveToPose;
import frc.robot.commands.DelayCommand;
import frc.robot.commands.SteerTowardsBall;
import frc.robot.commands.SteerTowardsHub;
import frc.robot.commands.TurnRobot;

/**
 * Basic auto which starts on right tarmac, grabs nearest ball, and shoots both
 */
public class ThreeBallAuto extends SequentialCommandGroup {
  /** Creates a new BasicAuto. */
  public ThreeBallAuto() {
    addCommands(
      new TwoBallAuto(),
      new TurnRobot(-167, false, 1),
      new TerminalBallAutoCommand()
    );
  }
}
