package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.SPI;

/**
 * This class cointains definitions for the robot's hardware devices.
 * These include components such as motor controllers and soleniods used by the subsystems.
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
        public static final int FL_STEER_ENCODER = 10;
        /** CAN ID for front-right drive falcon */
        public static final int FR_DRIVE_FALCON = 5;
        /** CAN ID for front-right steer falcon */
        public static final int FR_STEER_FALCON = 6;
        /** CAN ID for front-left steer encoder */
        public static final int FR_STEER_ENCODER = 11;
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
        public static final int BR_STEER_ENCODER = 12;
        public static final int LEADER_INTAKE_FALCON = 98;
        public static final int FOLLOWER_INTAKE_FALCON = 99;

    }

    public static final int INTAKE_SOLENOID_EXTEND = 96;
    public static final int INTAKE_SOLENOID_RETRACT = 97;

    public static final Mk4SwerveModuleHelper.GearRatio DRIVE_RATIO = Mk4SwerveModuleHelper.GearRatio.L1;

    public static final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200);

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
