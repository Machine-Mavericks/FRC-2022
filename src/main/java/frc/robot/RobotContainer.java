// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ReleaseBall;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SteerTowardsBall;
import frc.robot.subsystems.BallTargeting;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.HubTargeting;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveOdometry;

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
  public static final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  public static final Drivetrain drivetrain = new Drivetrain();
  public static final Gyro gyro = new Gyro();
  public static final LED led = new LED(RobotMap.PWMPorts.LED_STRIP);
  public static final SwerveOdometry odometry = new SwerveOdometry();
  
  // The robot's commands are defined here...
  private static final LEDCommand LEDCommand = new LEDCommand(led);   //Totally unused atm, might be handy for blinking or disco mode if we implement that?
  public static final Shooter m_shooter = new Shooter();
  public static final Lifter lifter = new Lifter();
  public static final Intake intake = new Intake();
  // public static final Limelight ballLimelight = new Limelight("camera name"); //TODO: set camera name
  // public static final Limelight hubLimelight = new Limelight("camera name");
  public static final BallTargeting ballTargeting = new BallTargeting();
  public static final HubTargeting hubTargeting = new HubTargeting();

  // The robot's subsystems are defined here...
  private static final ExampleCommand autoCommand = new ExampleCommand(exampleSubsystem);


  /** Initialise the container for the robot. Contains subsystems, OI devices, and commands. */
  public static void init() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
    // Initialise gyro to be forward-facing
    gyro.setCurrentYaw(0);
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

    OI.shootButton.whenPressed(new ShooterCommand());
    // TODO: Disable binding for competition use
    OI.zeroButton.whenPressed(() -> gyro.setCurrentYaw(0));
    OI.intakeButton.whileHeld(new IntakeCommand());
    OI.ballTrackingButton.whenHeld(new SteerTowardsBall());
    OI.releaseBallButton.whileHeld(new ReleaseBall());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}