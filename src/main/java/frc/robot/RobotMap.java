package frc.robot;


/**
 * This class cointains definitions for the robot's hardware devices.
 * These include components such as motor controllers and soleniods used by the
 * subsystems.
 */
public class RobotMap {

    /**
     * Inner class containing CANIDs
     */
    public static class CANID {
        /** CAN ID for front-left drive falcon */
        public static final int FL_DRIVE_FALCON = 3;
        /** CAN ID for front-left steer falcon */
        public static final int FL_STEER_FALCON = 4;
        /** CAN ID for front-left steer encoder */
        public static final int FL_STEER_ENCODER = 12;
        /** CAN ID for front-right drive falcon */
        public static final int FR_DRIVE_FALCON = 5;
        /** CAN ID for front-right steer falcon */
        public static final int FR_STEER_FALCON = 6;
        /** CAN ID for front-left steer encoder */
        public  static final int FR_STEER_ENCODER = 11;
        /** CAN ID for back-left drive falcon */
        public static final int BL_DRIVE_FALCON = 1;
        /** CAN ID for back-left steer falcon */
        public static final int BL_STEER_FALCON = 2;
        /** CAN ID for front-left steer encoder */
        public static final int BL_STEER_ENCODER = 9;
        /** CAN ID for back-right drive falcon */
        public static final int BR_DRIVE_FALCON = 7;
        /** CAN ID for back-right steer falcon */
        public static final int BR_STEER_FALCON = 8;
        /** CAN ID for front-left steer encoder */
        public static final int BR_STEER_ENCODER = 10;
    }


    /**
     * Function to initialise hardware
     * Should be called in {@link Robot#robotInit()}
     */
    public static void Init() {
    }
}
