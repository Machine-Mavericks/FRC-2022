package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

/**
 * This class cointains definitions for the robot's hardware devices.
 * These include components such as motor controllers and soleniods used by the subsystems.
 */
public class RobotMap {
    
    /**
     * Inner class containing CANIDs
     */
    private static class CANID {
        /** CAN ID for front-left drive falcon */
        static final int FL_DRIVE_FALCON = 2;
        /** CAN ID for front-left steer falcon */
        static final int FL_STEER_FALCON = 3;
        /** CAN ID for front-left steer encoder */
        static final int FL_STEER_ENCODER = 4;
        /** CAN ID for front-right drive falcon */
        static final int FR_DRIVE_FALCON = 5;
        /** CAN ID for front-right steer falcon */
        static final int FR_STEER_FALCON = 6;
        /** CAN ID for front-left steer encoder */
        static final int FR_STEER_ENCODER = 7;
        /** CAN ID for back-left drive falcon */
        static final int BL_DRIVE_FALCON = 8;
        /** CAN ID for back-left steer falcon */
        static final int BL_STEER_FALCON = 9;
        /** CAN ID for front-left steer encoder */
        static final int BL_STEER_ENCODER = 10;
        /** CAN ID for back-right drive falcon */
        static final int BR_DRIVE_FALCON = 11;
        /** CAN ID for back-right steer falcon */
        static final int BR_STEER_FALCON = 12;
        /** CAN ID for front-left steer encoder */
        static final int BR_STEER_ENCODER = 13;
    }

    public static final Mk4SwerveModuleHelper.GearRatio DRIVE_RATIO = Mk4SwerveModuleHelper.GearRatio.L1;

    public static final PigeonIMU pigeon = new PigeonIMU(0); // TODO: Set pigeon id properly

    /** Front-left swerve module */
    public static final SwerveModule frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(DRIVE_RATIO, CANID.FL_DRIVE_FALCON, CANID.FL_STEER_FALCON, CANID.FL_STEER_ENCODER, 0);
    /** Front-left swerve module */
    public static final SwerveModule frontRightModule = Mk4SwerveModuleHelper.createFalcon500(DRIVE_RATIO, CANID.FR_DRIVE_FALCON, CANID.FR_STEER_FALCON, CANID.FR_STEER_ENCODER, 0);
    /** Front-left swerve module */
    public static final SwerveModule backLeftModule = Mk4SwerveModuleHelper.createFalcon500(DRIVE_RATIO, CANID.BL_DRIVE_FALCON, CANID.BL_STEER_FALCON, CANID.BL_STEER_ENCODER, 0);
    /** Front-left swerve module */
    public static final SwerveModule backRightModule = Mk4SwerveModuleHelper.createFalcon500(DRIVE_RATIO, CANID.BR_DRIVE_FALCON, CANID.BR_STEER_FALCON, CANID.BR_STEER_ENCODER, 0);

    /**
     * Function to initialise hardware
     * Should be called in {@link Robot#robotInit()}
     */
    public static void Init(){
    }    
}
