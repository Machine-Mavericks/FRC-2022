// Shuffleboard
//
// Initializes and updates shuffleboard
// This module contains code for making and maintaining main shuffleboard page
// Other pages made by the individual subsystems as req'd

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.networktables.NetworkTableEntry;


public class ShuffleboardOI extends SubsystemBase {

    // example autonomous path shuffleboard selection boxes
    // true if selected, false if not
    // <add any other controls here that go on main shufflebard page
    private NetworkTableEntry m_toggleCenter;
    private NetworkTableEntry m_toggleOffCenter;
    private NetworkTableEntry m_toggleSimpleLeft;
    private NetworkTableEntry m_toggleSimpleRight;

    // other controls on main page
    private NetworkTableEntry m_timeLeft;

    /** Initializes the Shuffleboard
     * Creates the subsystem pages */
    public ShuffleboardOI() {

        // add autonomous commands to shuffleboard
        initializeMainShuffleboardPage();
    }

    /** Update Shuffleboard Pages. This method will be called once per scheduler run
     * (=50Hz) */
    @Override
    public void periodic() {

        // update main page
        // update remaining time in match (rounded to nearest second)
        m_timeLeft.setDouble(Math.round(Timer.getMatchTime()));
    }


    // -------------------- Shuffboard Methods --------------------


    /** Create main shuffleboard page
     * Typically contains autonomous commands and other top-level robot controls*/
    private void initializeMainShuffleboardPage() {

        // Create Main Tab in Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Auto");

        // add autonomous commands to page - example adds toggle switches
        m_toggleCenter = tab.add("Center", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(0, 0)
                .withSize(1, 1).getEntry();
        m_toggleOffCenter = tab.add("OffCenter", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(1, 0)
                .withSize(1, 1).getEntry();
        m_toggleSimpleLeft = tab.add("SimpleLeft", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(2, 0)
                .withSize(1, 1).getEntry();
        m_toggleSimpleRight = tab.add("SimpleRight", false).withWidget(BuiltInWidgets.kToggleSwitch).withPosition(3, 0)
                .withSize(1, 1).getEntry();

        // add match time remaining in autonomous/teleop part of match (seconds)
        ShuffleboardLayout l1 = tab.getLayout("Timer", BuiltInLayouts.kList);
        l1.withPosition(0, 2);
        l1.withSize(1, 2);
        m_timeLeft = l1.add("TimeLeft", 0.0).getEntry();
    }

    // returns position of autonomous commands on shuffleboard
    // typically called by Robot AutonomousInit to select auto path to be followed
    // returns true if selected, false if not
    // TODO <to be revised for 2022 robot>

    /** Get status of Auto Center toggle switch */
    public boolean getAutoCommandCenter() {
        return m_toggleCenter.getBoolean(false);
    }

    /** Get status of Auto offCenter toggle switch */
    public boolean getAutoCommandOffCenter() {
        return m_toggleOffCenter.getBoolean(false);
    }

    /** Get status of Auto SimpleLeft toggle switch */
    public boolean getAutoCommandSimpleLeft() {
        return m_toggleSimpleLeft.getBoolean(false);
    }

    /** Get status of Auto SimpleRight toggle switch */
    public boolean getAutoCommandSimpleRight() {
        return m_toggleSimpleRight.getBoolean(false);
    }

} // end class ShuffleboardOI