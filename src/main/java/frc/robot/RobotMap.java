package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * This class cointains definitions for the robot's hardware devices.
 * These include components such as motor controllers and soleniods used by the
 * subsystems.
 */
public class RobotMap {

    /**
     * Inner class containing CANIDs
     */
    private static class CANID {
        /** CAN ID for front-left drive falcon */
        static final int FL_DRIVE_FALCON = 3;
        /** CAN ID for front-left steer falcon */
        static final int FL_STEER_FALCON = 4;
        /** CAN ID for front-left steer encoder */
        static final int FL_STEER_ENCODER = 12;
        /** CAN ID for front-right drive falcon */
        static final int FR_DRIVE_FALCON = 5;
        /** CAN ID for front-right steer falcon */
        static final int FR_STEER_FALCON = 6;
        /** CAN ID for front-left steer encoder */
        static final int FR_STEER_ENCODER = 11;
        /** CAN ID for back-left drive falcon */
        static final int BL_DRIVE_FALCON = 1;
        /** CAN ID for back-left steer falcon */
        static final int BL_STEER_FALCON = 2;
        /** CAN ID for front-left steer encoder */
        static final int BL_STEER_ENCODER = 9;
        /** CAN ID for back-right drive falcon */
        static final int BR_DRIVE_FALCON = 7;
        /** CAN ID for back-right steer falcon */
        static final int BR_STEER_FALCON = 8;
        /** CAN ID for front-left steer encoder */
        static final int BR_STEER_ENCODER = 10;
    }

    public static final Mk4SwerveModuleHelper.GearRatio DRIVE_RATIO = Mk4SwerveModuleHelper.GearRatio.L1;

    public static final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200);

    static ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    /** Front-left swerve module */
    public static final SwerveModule frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            DRIVE_RATIO, CANID.FL_DRIVE_FALCON, CANID.FL_STEER_FALCON, CANID.FL_STEER_ENCODER, -Math.toRadians(155+180));
    /** Front-left swerve module */
    public static final SwerveModule frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            DRIVE_RATIO, CANID.FR_DRIVE_FALCON, CANID.FR_STEER_FALCON, CANID.FR_STEER_ENCODER, -Math.toRadians(94+180));
    /** Front-left swerve module */
    public static final SwerveModule backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            DRIVE_RATIO, CANID.BL_DRIVE_FALCON, CANID.BL_STEER_FALCON, CANID.BL_STEER_ENCODER, -Math.toRadians(200+180));
    /** Front-left swerve module */
    public static final SwerveModule backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            DRIVE_RATIO, CANID.BR_DRIVE_FALCON, CANID.BR_STEER_FALCON, CANID.BR_STEER_ENCODER, -Math.toRadians(135+180));

    /**
     * Function to initialise hardware
     * Should be called in {@link Robot#robotInit()}
     */
    public static void Init() {
    }
}
