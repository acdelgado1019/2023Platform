package frc.robot.Commands.Autonomous;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.MecanumDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import java.util.List;
import frc.robot.Constants;
import frc.robot.HDD;
import frc.robot.Robot;

public class AutoMethods {
    public static MecanumDriveKinematicsConstraint autoVoltageConstraint;
    public static TrajectoryConfig config;
    public static Trajectory trajectory;
    public static RamseteCommand ramseteCommand;

    // Create a voltage constraint to ensure we don't accelerate too fast
    public static MecanumDriveKinematicsConstraint getConstraint(){
        autoVoltageConstraint =
        new MecanumDriveKinematicsConstraint(
            Robot.drivetrain.kinematics, 3
        );   
        return autoVoltageConstraint;
    }

    // Create config for trajectory
    public static TrajectoryConfig getTrajectoryConfig(){
        config =
        new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Robot.drivetrain.kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
        return config;
    }

    // Trajectory to follow.  All units in meters.
    public static Trajectory getTrajectory(){
        switch (HDD.desiredMode){
            case TEMPLATE :                
                trajectory =
                TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing goal
                new Pose2d(5.9, 4.1, new Rotation2d(0)),
                // Pass through ball 2 and 3
                List.of(new Translation2d(4.5, 3.3), new Translation2d(4.5, 3.5)),
                // Turn to goal and come in range
                new Pose2d(5.5, 3.9, new Rotation2d(Math.PI/8)),
                // Pass config
                config);
                break;
        }
        return trajectory;
    }

    // Reset odometry to the starting pose of the trajectory.
    public static void resetOdometry(Trajectory trajectory){
        Pose2d pose = trajectory.getInitialPose();
        Robot.drivetrain.resetOdometry(pose);
    }

    public static Command runTrajectory(){
        trajectory = getTrajectory();
        return ramseteCommand.andThen(() -> Robot.drivetrain.driveVolts(0,0,0, 0));
    }
}
