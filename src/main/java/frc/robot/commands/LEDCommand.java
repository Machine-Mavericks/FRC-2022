// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Command controls LEDs to display required information to driver */
public class LEDCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  /**
   * Creates a new LEDCommand.
   */
  public LEDCommand() {
    addRequirements(RobotContainer.LEDStrip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  int counter = 0;
  @Override
  public void execute() {
    
    // counter used to create blink rate of ~2.5Hz
    counter++;
    if (counter>=20)
      counter=0;
    
    // use counter to interleave blinks of LEDs as req'd
    if (counter==0)
    {
      if (RobotContainer.hubTargeting.IsTarget())
        RobotContainer.LEDStrip.SetEntireStripColorRGB(255, 255, 0);
      else
         RobotContainer.LEDStrip.SetEntireStripColorRGB(0, 0, 0);
    }

    else if (counter==10)
    {
      if (RobotContainer.ballTargeting.IsBall())
        {
          if (DriverStation.getAlliance() == Alliance.Red)
            RobotContainer.LEDStrip.SetEntireStripColorRGB(255, 0, 0);
          else
             RobotContainer.LEDStrip.SetEntireStripColorRGB(0, 0, 255);
        }
      else
        RobotContainer.LEDStrip.SetEntireStripColorRGB(0, 0, 0);
    }
    
    //else
    //  RobotContainer.LEDStrip.SetEntireStripColorRGB(0, 0, 0);


}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.LEDStrip.SetEntireStripColorRGB(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
