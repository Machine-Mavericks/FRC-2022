// Delay Command //

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;


public class DelayCommand extends CommandBase {
    //

    private double m_TIMER_END = 0;
    private double m_delayTimer;

    // Delay command function //
    public DelayCommand(double delay) {
        m_delayTimer = delay; 
    }

    @Override
    public void initialize() {
        m_TIMER_END = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_TIMER_END += 0.02;
    }

    // Returns false when the command should end.
    @Override
    public boolean isFinished() {
        if (m_delayTimer > m_TIMER_END) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
    }
}
