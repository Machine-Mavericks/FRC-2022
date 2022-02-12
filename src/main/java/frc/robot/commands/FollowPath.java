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




public class FollowPath extends CommandBase {

    private Drivetrain m_drivetrain = RobotContainer.drivetrain;
    private Trajectory robotRelativeTrajectory;
    private Trajectory trajectory;
    private double temp;

    private Timer timer = new Timer();
    private final SwerveOdometry m_odometry = RobotContainer.odometry;

    // TODO: Tune these
    private double p = 1;
    private double i = 0;
    private double d = 0.06;

    private Pose2d odometryPose = new Pose2d();
    private Rotation2d desiredAngle; // = new Rotation2d(0,0);
    private ChassisSpeeds speeds = new ChassisSpeeds();

    private HolonomicDriveController driveController;

    // Measured in m/s and m/s/s
    private final double MAX_VELOCITY = 1.0;
    private final double MAX_ACCELERATION = 0.5;

    // Input the name of the generated path in PathPlanner
    public FollowPath(double[][] points, double startAngle, double endAngle, double startVelocity,
            double endVelocity) {
        trajectory = calculateTrajectory(points, startAngle, endAngle, startVelocity, endVelocity);

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
        driveController = new HolonomicDriveController(
                new PIDController(p, i, d), new PIDController(p, i, d), rotationController);
        driveController.setEnabled(true);

        // Start timer when path begins
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get next trajectory state from our path
        State targetPathState = trajectory.sample(timer.get());

        // set robot's angle - for now choose same angle as the path. We can improve this after we get basic path working
        desiredAngle = targetPathState.poseMeters.getRotation();

        // get our current odeometry Pose
        odometryPose = m_odometry.getPose2d();

        // determine robot chassis speeds
        speeds = driveController.calculate(odometryPose, targetPathState, desiredAngle);

        //swap x and y
        /*temp = -speeds.vxMetersPerSecond;
        speeds.vxMetersPerSecond = -speeds.vyMetersPerSecond;
        speeds.vyMetersPerSecond = temp;*/

        speeds.omegaRadiansPerSecond = 0.0;
        speeds.vyMetersPerSecond = 1.0;
        speeds.vxMetersPerSecond = 0.0;

        // instruct drive system to move robot
        m_drivetrain.setChassisSpeeds(speeds);

       RobotContainer.shuffleboard.time.setDouble(timer.get());
        RobotContainer.shuffleboard.x.setDouble(targetPathState.poseMeters.getX());
        RobotContainer.shuffleboard.y.setDouble(targetPathState.poseMeters.getY());
        RobotContainer.shuffleboard.rot.setDouble(trajectory.getTotalTimeSeconds());

        RobotContainer.shuffleboard.x1.setDouble(odometryPose.getX());
        RobotContainer.shuffleboard.y1.setDouble(odometryPose.getY());
        //RobotContainer.shuffleboard.rot1.setDouble(odometryPose.getRotation().getDegrees());

        RobotContainer.shuffleboard.speedX.setDouble(speeds.vxMetersPerSecond);
        RobotContainer.shuffleboard.speedY.setDouble(speeds.vyMetersPerSecond);

    }

    /* Optional improved functionality
    // Read the path with the given name from the PathPlanner
    private PathPlannerTrajectory getTrajectory(String pathName) {
        PathPlannerTrajectory currentTrajectory = PathPlanner.loadPath(pathName, MAX_VELOCITY, MAX_ACCELERATION);

        return currentTrajectory;
    } */

    // Calculate trajectory manually with a clamped cubic spline
    private Trajectory calculateTrajectory(double[][] points, double startAngle, double endAngle, double startVelocity,
            double endVelocity) {

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
        trajectoryConfig.setStartVelocity(startVelocity);
        trajectoryConfig.setEndVelocity(endVelocity);

        robotRelativeTrajectory = TrajectoryGenerator.generateTrajectory(startPose, path, endPose, trajectoryConfig);

        // translate/rotate path so it starts the robot's current position/angle
        return robotRelativeTrajectory.transformBy(new Transform2d(m_odometry.getPose2d(), startPose));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Translation2d zeroPoint = new Translation2d(0.0, 0.0);
        m_drivetrain.drive(zeroPoint, 0.0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // we are finished when the time spent in this command is >= duration of path (in seconds)
        if (timer.get() >= trajectory.getTotalTimeSeconds())
            return true;
        else
            return false;
    }

}