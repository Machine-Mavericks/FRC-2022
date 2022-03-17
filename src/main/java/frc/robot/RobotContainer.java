// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.BallCameraAutoTilt;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.LowerShooter;
import frc.robot.commands.ReleaseBall;
import frc.robot.commands.RetractClimber;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SteerTowardsBall;
import frc.robot.commands.SteerTowardsHub;
import frc.robot.commands.TiltShooter;
import frc.robot.commands.autonomous.LowBallAuto;
import frc.robot.commands.autonomous.OneBallAuto;
import frc.robot.commands.autonomous.ThreeBallAuto;
import frc.robot.commands.autonomous.TwoBallAuto;
import frc.robot.subsystems.BallTargeting;
import frc.robot.subsystems.CameraTilt;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.HubTargeting;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDBlinkin;
import frc.robot.subsystems.Lifter;
import frc.robot.subsystems.PowerPanel;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveOdometry;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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
  // public static final LED LEDStrip = new LED(RobotMap.PWMPorts.LED_STRIP1);
  public static final LEDBlinkin LEDStrip = new LEDBlinkin();
  public static final Shooter m_shooter = new Shooter();
  public static final Lifter lifter = new Lifter();
  public static final Intake intake = new Intake();
  public static final BallTargeting ballTargeting = new BallTargeting();
  public static final HubTargeting hubTargeting = new HubTargeting();
  public static final CameraTilt cameraTilt = new CameraTilt();
  public static final Climber climber = new Climber();

  /**
   * Initialise the container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public static void init() {
    drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
    LEDStrip.setDefaultCommand(new LEDCommand());
    cameraTilt.setDefaultCommand(new BallCameraAutoTilt());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private static void configureButtonBindings() {
    // OI.LEDButton.whenPressed(() -> led.SetEntireStripColorRGB(255, 0, 0));

    OI.highSpeedButton.whenPressed(new ShooterCommand()); //TODO; change to while held
    // TODO: Disable binding for competition use
    OI.zeroButton.whenPressed(() -> gyro.resetGyro());
    // OI.zeroButton.whenPressed(new RecordCurrentPose2d());
    OI.intakeButton.whileHeld(new IntakeCommand());
    OI.ballTrackingButton.whenHeld(new SteerTowardsBall(false, 20.0));
    OI.hubTrackingButton.whenHeld(new SteerTowardsHub());
    OI.releaseBallButton.whileHeld(new ReleaseBall());

    OI.tiltShooterButton.whenPressed(new TiltShooter());
    OI.lowerShooterButton.whenPressed(new LowerShooter());
    // OI.testRobotRelativePath.whileHeld(new AutoDriveToPose(0.5, 0.20));

    if (HAL.getMatchTime() > 119.0){
      OI.extendClimberButton.whileHeld(new ExtendClimber());
      OI.retractClimberButton.whileHeld(new RetractClimber());}
    

    // OI.testRobotRelativePath.whileHeld(new AutoDriveToPose(new Pose2d(0, 0, new
    // Rotation2d(0)), 0.35, 0.15, 20.0));
    // new TurnRobot(45.0,false,2.0));//new SampleAutoCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    if (RobotContainer.shuffleboard.m_selectedPath == 0) {
      return new TwoBallAuto();
    } else if (RobotContainer.shuffleboard.m_selectedPath == 1) {
      return new ThreeBallAuto();
    } else if (RobotContainer.shuffleboard.m_selectedPath == 3) {
      return new OneBallAuto();
    } else {
      return new LowBallAuto();
    }
  }
}