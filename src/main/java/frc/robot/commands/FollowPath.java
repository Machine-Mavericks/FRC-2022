// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveOdometry;


/** Command to Follow Path */
public class FollowPath extends CommandBase {

    private Drivetrain m_drivetrain = RobotContainer.drivetrain;
    private Trajectory trajectory;

    private Timer timer;
    private final SwerveOdometry m_odometry = RobotContainer.odometry;

    // TODO: Tune these
    // PIDs gains for X and Y position controllers
    private double p = 5;
    private double i = 0;
    private double d = 0;  //0.06;

    private Pose2d odometryPose = new Pose2d();
    private Rotation2d desiredAngle; 
    private ChassisSpeeds speeds = new ChassisSpeeds();

    private HolonomicDriveController driveController;

    private double m_endRobotAngle;
    private double m_RobotRotationRate;
    private double m_RobotAngle;

    // Measured in m/s and m/s/s
    private final double MAX_VELOCITY = 1.0;
    private final double MAX_ACCELERATION = 0.5;

    /** Follow Generic Path
     * Inputs:  points - coordinates that define path
     *          startAngle - starting angle of path
     *          endAngle - ending angle of path
     *          startSpeed - starting speed of robot (m/s)
     *          endSpeed - ending speed of robot (m/s)
     */
    public FollowPath(double[][] points, double startAngle, double endAngle, double startSpeed,
            double endSpeed, double endRobotAngle, boolean reverse, boolean rotatepath) {
        trajectory = calculateTrajectory(points,
                                        startAngle,
                                        endAngle,
                                        startSpeed,
                                        endSpeed,
                                        reverse,
                                        rotatepath);

        // set up timer to track time in the path
        timer = new Timer();

        m_endRobotAngle = endRobotAngle;

        addRequirements(m_drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Create necessary profiled PID controller and configure it to be used with the
        // holonomic controller
        ProfiledPIDController rotationController = new ProfiledPIDController(
                2.5,
                0.0,
                0.0,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

        // Create main holonomic drive controller
        // Note: From drive testing Feb10/2022, y axis in path controller is reverse from odometry
        // use negative y-axis controller gains to avoid positive feedback!
        driveController = new HolonomicDriveController(
                new PIDController(p, i, d), new PIDController(-p, -i, -d), rotationController);
        driveController.setEnabled(true);

        // beginning angle of robot - set to urrent angle
        m_RobotAngle = RobotContainer.gyro.continuousYaw();
        
        // robot rotation rate
        double rotatetime = trajectory.getTotalTimeSeconds();
        if (rotatetime!=0.0)
            m_RobotRotationRate = (m_endRobotAngle-m_RobotAngle)/rotatetime;
       else
            m_RobotRotationRate = 0.0;

       // beginning angle of robot - set to urrent angle
            m_RobotAngle = RobotContainer.gyro.continuousYaw();

        // Start timer when path begins
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get next trajectory state from our path
        State targetPathState = trajectory.sample(timer.get());

        // update our target robot rotation angle
        m_RobotAngle += m_RobotRotationRate*0.02;

        // set robot's angle - for now choose same angle as the path. We can improve this after we get basic path working
        desiredAngle = new Rotation2d(m_RobotAngle*3.1415/180.0); // targetPathState.poseMeters.getRotation();

        // get our current odeometry Pose
        odometryPose = m_odometry.getPose2d();

        // determine robot chassis speeds
        speeds = driveController.calculate(odometryPose, targetPathState, desiredAngle);
        
        // instruct drive system to move robot
        //m_drivetrain.setChassisSpeeds(speeds);
        m_drivetrain.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                            speeds.omegaRadiansPerSecond,
                            true);
    }


    /** Calculate trajectory manually with a clamped cubic spline */
    private Trajectory calculateTrajectory(double[][] points, double startAngle, double endAngle, double startSpeed,
            double endSpeed, boolean reverse, boolean rotatepath) {

        Pose2d startPose = new Pose2d(points[0][0], points[0][1], Rotation2d.fromDegrees(startAngle));
        Pose2d endPose = new Pose2d(points[points.length - 1][0], points[points.length - 1][1],
                Rotation2d.fromDegrees(endAngle));

        ArrayList<Translation2d> path = new ArrayList<>();

        // If the desired set of points contains at least one waypoint, add them to the
        // path object
        if (points.length > 2) {
            for (int i = 1; i < path.size() - 1; i++) {
                Translation2d point = new Translation2d(points[i][0], points[i][1]);
                path.add(point);
            }
        }

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION);
        trajectoryConfig.setStartVelocity(startSpeed);
        trajectoryConfig.setEndVelocity(endSpeed);
        trajectoryConfig.setReversed(reverse);

        // create our trajectory
        Trajectory robotRelativeTrajectory = TrajectoryGenerator.generateTrajectory(startPose, path, endPose, trajectoryConfig);

       
        Translation2d ken = new Translation2d(m_odometry.getPose2d().getX() - startPose.getX(),
                                               m_odometry.getPose2d().getY() - startPose.getY());
        Rotation2d ken2 = new Rotation2d(0.0);
        if (rotatepath)
            ken2 = m_odometry.getPose2d().getRotation();

        Transform2d ken3 = new Transform2d(ken, ken2);

        // return generaed trajectory
        return robotRelativeTrajectory.transformBy(ken3); 
        
        //return robotRelativeTrajectory.transformBy(new Transform2d(m_odometry.getPose2d(), startPose));
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        Translation2d zeroPoint = new Translation2d(0.0, 0.0);
        m_drivetrain.drive(zeroPoint, 0.0, true);
    }

    /** Returns true when the command should end. */
    @Override
    public boolean isFinished() {
        // we are finished when the time spent in this command is >= duration of path (in seconds)
        if (timer.get() >= trajectory.getTotalTimeSeconds())
            return true;
        else
            return false;
    }

} // end FollowPath command