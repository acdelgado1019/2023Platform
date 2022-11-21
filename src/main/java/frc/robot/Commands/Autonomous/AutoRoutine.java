package frc.robot.Commands.Autonomous;

import frc.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import java.util.List;

public class AutoRoutine {

    /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command runAutonomous() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    MecanumControllerCommand mecanumControllerCommand =
        new MecanumControllerCommand(
            trajectory,
            Robot.drivetrain::getPose,
            DriveConstants.kFeedforward,
            DriveConstants.kinematics,

            // Position contollers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),

            // Needed for normalizing wheel speeds
            AutoConstants.kMaxSpeedMetersPerSecond,

            // Velocity PID's
            new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
            new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
            new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
            new PIDController(DriveConstants.kPRearRightVel, 0, 0),
            Robot.drivetrain::getWheelSpeeds,
            Robot.drivetrain::setDriveMotorControllersVolts, // Consumer for the output motor voltages
            Robot.drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    Robot.drivetrain.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return mecanumControllerCommand.andThen(() -> Robot.drivetrain.drive(0, 0, 0, false));
  }
    
}
