// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoShootCommand extends CommandBase {

  /**
   * 
   */
  public static final DoubleSupplier LOW_SPEED = () -> 1750;
  public static final DoubleSupplier HIGH_SPEED = () -> 3300;
  
  private DoubleSupplier flywheelSpeed;
  private boolean ballDetected = false;
  public LinearFilter liftLimitFiltered = LinearFilter.movingAverage(10);
  private boolean limPressed = false;

  /**
   * Create a new command to shoot a ball in auto
   * Note that this does not apply any targeting consideration
   * @param flywheelSpeed Double supplier to be used for getting the flywheel speed, in RPM. 
   * Defaults for low and high are provided by the class as {@link #LOW_SPEED}, and {@link #HIGH_SPEED} respetively
  */
  public AutoShootCommand(DoubleSupplier flywheelSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_shooter);
    addRequirements(RobotContainer.lifter);
    this.flywheelSpeed = flywheelSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballDetected = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set shooter speed based on supplied value
    RobotContainer.m_shooter.setShooterSpeed(flywheelSpeed.getAsDouble());
    if(RobotContainer.m_shooter.getShooterSpeed() >= flywheelSpeed.getAsDouble()*0.95){
      RobotContainer.lifter.liftBalls();
    }

    // Update the limit switch filter
    limPressed = liftLimitFiltered.calculate(!RobotContainer.lifter.liftLimit.get() ? 1 : 0) > 0.5;
    // Set flag when ball detected at limit switch
    if(limPressed) ballDetected = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_shooter.setShooterSpeed(RobotContainer.m_shooter.ChosenIdleSpeed.getDouble(0));
    RobotContainer.lifter.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Limit switch is true once ball has left
    return !limPressed && ballDetected;
  }
}
