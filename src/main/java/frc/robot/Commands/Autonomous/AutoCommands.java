package frc.robot.Commands.Autonomous;

import frc.robot.Robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;

public class AutoCommands {
    public static Trajectory trajectoryA;

    public static Command drivetrainMotion(Trajectory trajectory) {
    
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
