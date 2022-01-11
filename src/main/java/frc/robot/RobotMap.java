package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * This class cointains definitions for the robot's hardware devices.
 * These include components such as motor controllers and soleniods used by the subsystems.
 */
public class RobotMap {
    
    /**
     * Inner class containing CANIDs
     */
    private static class CANID {
        /** CAN ID for first example falcon */
        static final int EXAMPLE_FALCON_1 = 0;
        /** CAN ID for second example falcon */
        static final int EXAMPLE_FALCON_2 = 1;
    }

    /** First example falcon. Mapped to {@link CANID#EXAMPLE_FALCON_1} */
    public static final WPI_TalonFX exampleFalcon1 = new WPI_TalonFX(CANID.EXAMPLE_FALCON_1);
    /** Second example falcon. Mapped to {@link CANID#EXAMPLE_FALCON_2} */
    public static final WPI_TalonFX exampleFalcon2 = new WPI_TalonFX(CANID.EXAMPLE_FALCON_2);

    /**
     * Function to initialise hardware
     * Should be called in {@link Robot#robotInit()}
     */
    public static void Init(){
        // Configure motor controllers here
        exampleFalcon2.follow(exampleFalcon2);
    }    
}
