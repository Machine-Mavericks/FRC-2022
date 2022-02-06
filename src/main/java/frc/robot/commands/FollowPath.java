// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveOdometry;
import frc.robot.RobotContainer;

public class FollowPath extends CommandBase {
    private Drivetrain m_drivetrain = RobotContainer.drivetrain;
    private Trajectory robotRelativeTrajectory;
    private Trajectory trajectory;

    private Timer timer = new Timer();
    private final SwerveOdometry m_odometry = RobotContainer.odometry;

    // TODO: Tune these
    private double p = 6;
    private double i = 0;
    private double d = 0.06;

    private MMState state = new MMState();
    private Pose2d odometryPose = new Pose2d();
    private Rotation2d desiredAngle;
    private ChassisSpeeds speeds = new ChassisSpeeds();

    private HolonomicDriveController driveController;

    // Measured in m/s and m/s/s
    private final double MAX_VELOCITY = 5;
    private final double MAX_ACCELERATION = 4;

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
        // To access the desired angle at the current state, the Trajectory.State must
        // be converted to an MMState
        state = MMState.fromState(trajectory.sample(timer.get()));
        desiredAngle = state.holonomicRotation;
        odometryPose = m_odometry.getPose2d();
        speeds = driveController.calculate(odometryPose, (Trajectory.State) state, desiredAngle);

        // Send the desired speeds to the m_drivetrain
        m_drivetrain.setChassisSpeeds(speeds);
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
        return false;
    }

    private static class MMState extends PathPlannerState {

        static MMState fromState(State s){
            MMState out = new MMState();
            out.timeSeconds = s.timeSeconds;
            out.velocityMetersPerSecond = s.velocityMetersPerSecond;
            out.accelerationMetersPerSecondSq = s.accelerationMetersPerSecondSq;
            out.poseMeters = s.poseMeters;
            out.curvatureRadPerMeter = s.curvatureRadPerMeter;
            return out;
        }
    }
}