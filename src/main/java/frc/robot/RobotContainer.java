// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SampleAutoCommand;
import frc.robot.commands.ReleaseBall;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SteerTowardsBall;
import frc.robot.commands.SteerTowardsHub;
import frc.robot.commands.autonomous.BasicAuto;
import frc.robot.commands.LEDCommand;
import frc.robot.subsystems.BallTargeting;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.HubTargeting;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveOdometry;
import frc.robot.subsystems.PowerPanel;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // Create robot's shuffboard operator interface
  public static final ShuffleboardOI shuffleboard = new ShuffleboardOI();

  // The robot's subsystems are defined here...
  public static final Gyro gyro = new Gyro();
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final SwerveOdometry odometry = new SwerveOdometry();
  public static final PowerPanel panel = new PowerPanel();
  public static final LED LEDStrip = new LED(RobotMap.PWMPorts.LED_STRIP1);
  public static final Shooter m_shooter = new Shooter();
  public static final Lifter lifter = new Lifter();
  public static final Intake intake = new Intake();
  public static final BallTargeting ballTargeting = new BallTargeting();
  public static final HubTargeting hubTargeting = new HubTargeting();

  /** Initialise the container for the robot. Contains subsystems, OI devices, and commands. */
  public static void init() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
    LEDStrip.setDefaultCommand(new LEDCommand());
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private static void configureButtonBindings() {
    //OI.LEDButton.whenPressed(() -> led.SetEntireStripColorRGB(255, 0, 0));

    OI.shootButton.whileHeld(new ShooterCommand());
    // TODO: Disable binding for competition use
    OI.zeroButton.whenPressed(() -> gyro.resetGyro());
    OI.intakeButton.whileHeld(new IntakeCommand());
    OI.ballTrackingButton.whenHeld(new SteerTowardsBall(false, 20.0));
    OI.hubTrackingButton.whenHeld(new SteerTowardsHub());
    OI.releaseBallButton.whileHeld(new ReleaseBall());
  
    OI.testRobotRelativePath.whileHeld(new SampleAutoCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new BasicAuto();
  }
}