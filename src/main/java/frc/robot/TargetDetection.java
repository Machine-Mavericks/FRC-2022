// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Target detection and estimation functions
// These functions may use various robot systems as required 

package frc.robot;

/** Add your docs here. */
public class TargetDetection {

    TargetDetection target;

    // minimum chevron detection area (# square pixels)
    public static final double MIN_HEX_DETECTION_AREA = 1000.0;
    public static final double MIN_HEX_VERT_SIZE = 20.0;

    boolean Detected; // true if target has been detected
    int TargetType; // type of target (0=chevron, 1=ball)
    double Area; // area of detected target (in # sq pixels)
    double XAngle; // angle (deg) of target relative to center-line of camera
    double XDistance; // estimated x distance (in) of target from camera (chevron only)
    double ZDistance; // estimated z distance (in) of target from camera

    TargetDetection() // defaults
    {
        Detected = false;
    }

    public TargetDetection GetTargetEstimation() {

        // assume no target detected unless proven otherwise
        target.Detected = false;

        // determine if camera has acquired a target
        boolean detected = RobotContainer.limelight.isTargetPresent();

        // get target side lengths
        double vert = RobotContainer.limelight.getVerticalSideLength();
        double hor = RobotContainer.limelight.getHorizontalSideLength();

        // get target area of target (in sq pixels)
        double area = vert * hor;

        if (detected == true && area > MIN_HEX_DETECTION_AREA && vert > MIN_HEX_VERT_SIZE) {
            target.Detected = true;
            target.TargetType = 0;
            target.Area = area;
            target.XAngle = RobotContainer.limelight.getHorizontalTargetOffsetAngle();
            target.ZDistance = (235.0 / vert);
        }

        // return target data
        return target;
    }

}
