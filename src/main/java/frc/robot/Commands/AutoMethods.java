package frc.robot.Commands;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import java.util.List;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robot.AutoSection;
import frc.robot.Robot.DesiredMode;

public class AutoMethods {
    public static DifferentialDriveVoltageConstraint autoVoltageConstraint;
    public static TrajectoryConfig config;
    public static Trajectory trajectory;
    public static RamseteCommand ramseteCommand;

    public static void limelightShoot()
    {
        while(Robot.limelight.getTV() != 0 && Math.abs(Robot.limelight.getTX()) > 3)
        {
            Robot.drivetrain.hubTrack();
        }
        Robot.drivetrain.setLeftDrivetrain(0);
        Robot.drivetrain.setRightDrivetrain(0);
        Robot.intake.setTrigger(Constants.TRIGGER_SPEED);
        Robot.ledStrip.solid(60);
        Timer.delay(1);
        Robot.intake.setTrigger(0);
    }

    // Create a voltage constraint to ensure we don't accelerate too fast
    public static DifferentialDriveVoltageConstraint getConstraint(){
        autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
                Robot.drivetrain.kinematics,
                10
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
        switch (Robot.desiredMode){
            case BACK_UP_BLUE :
                Robot.drivetrain.initPose = 0.0;
                config.setReversed(true);
                trajectory =
                TrajectoryGenerator.generateTrajectory(
                new Pose2d(5.9, 4.1, new Rotation2d(0)),
                List.of(new Translation2d(4.5, 4.1), new Translation2d(3, 4.1)),
                new Pose2d(1, 4.1, new Rotation2d(0)),
                // Pass config
                config);
                break;
            case ONE_BALL_BLUE :                
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
            case TWO_BALL_BLUE :  
                Robot.drivetrain.initPose = 136.5;
                trajectory =
                TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing ball
                new Pose2d(6.1, 5.2, new Rotation2d(136.5*Math.PI*2/360)),
                // pass through ball location
                List.of(new Translation2d(4.5, 6.7), new Translation2d(5, 7.2)),
                // Loop back around, facing goal
                new Pose2d(6.1, 6.2, new Rotation2d(-50*Math.PI*2/360)),
                // Pass config
                config);
                break;
            case THREE_BALL_BLUE :  
                Robot.drivetrain.initPose = 80;
                trajectory =
                TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing goal
                new Pose2d(7.8, 1.9, new Rotation2d(80*Math.PI*2/360)),
                // Pass through ball 2 and 3
                List.of(new Translation2d(7.7, 0.7), new Translation2d(5.1, 1.8)),
                // Turn to goal and come in range
                new Pose2d(5.5, 2.2, new Rotation2d(-325*Math.PI*2/360)),
                // Pass config
                config);
                break;
            case BACK_UP_RED :  
                Robot.drivetrain.initPose = 180.0;
                config.setReversed(true);
                trajectory =
                TrajectoryGenerator.generateTrajectory(
                new Pose2d(10.55, 4.1, new Rotation2d(Math.PI)),
                List.of(new Translation2d(11, 4.1), new Translation2d(13, 4.1)),
                new Pose2d(15.5, 4.1, new Rotation2d(Math.PI)),
                // Pass config
                config);
                break;
            case ONE_BALL_RED :
                Robot.drivetrain.initPose = 180;
                trajectory =
                TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing goal
                new Pose2d(10.55, 4.1, new Rotation2d(Math.PI)),
                // Pass through ball 2 and 3
                List.of(new Translation2d(12.1, 5.0), new Translation2d(12.1, 4.8)),
                // Turn to goal and come in range
                new Pose2d(10.55, 4.1, new Rotation2d(9*Math.PI/8)),
                // Pass config
                config);
                break;
            case TWO_BALL_RED :
                Robot.drivetrain.initPose = -43.5;
                trajectory =
                TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing ball
                new Pose2d(10.4, 3.0, new Rotation2d(-43.5*Math.PI*2/360)),
                // pass through ball location
                List.of(new Translation2d(12, 1.5), new Translation2d(11.5, 1)),
                // Loop back around, facing goal
                new Pose2d(10.5, 1.8, new Rotation2d(130.5*Math.PI*2/360)),
                // Pass config
                config);
                break;
            case THREE_BALL_RED :
                Robot.drivetrain.initPose = -100;
                trajectory =
                TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing goal
                new Pose2d(8.75, 6.4, new Rotation2d(-100*Math.PI*2/360)),
                // Pass through ball 2 and 3
                List.of(new Translation2d(8.7, 7.5), new Translation2d(11.3, 6.6)),
                // Turn to goal and come in range
                new Pose2d(11, 6.3, new Rotation2d(-145*Math.PI*2/360)),
                // Pass config
                config);
                break;
        }
        return trajectory;
    }

    public static RamseteCommand getRamsete(){
        ramseteCommand =
        new RamseteCommand(
            trajectory,
            Robot.drivetrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
            Robot.drivetrain.kinematics,
            Robot.drivetrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            Robot.drivetrain::tankDriveVolts,
            Robot.drivetrain);
        return ramseteCommand;
    }

    // Reset odometry to the starting pose of the trajectory.
    public static void resetOdometry(Trajectory trajectory){
        Pose2d pose = trajectory.getInitialPose();
        Robot.drivetrain.resetOdometry(pose);
    }

    public static Command runRamsete(){
        autoVoltageConstraint = getConstraint();
        config = getTrajectoryConfig();
        trajectory = getTrajectory();
        ramseteCommand = getRamsete();
        resetOdometry(trajectory);
        return ramseteCommand.andThen(() -> Robot.drivetrain.tankDriveVolts(0, 0));
    }

    public static void runAutonomous(double timeCheck){
        switch(Robot.autoSection){
            case STARTUP :
                var pidOutput = Robot.intake.Lift_controller.calculate(
                Robot.intake.getEncoder(), 
                Units.degreesToRadians(Constants.loILPositionDeg));
                Robot.intake.setIntakeLift(pidOutput);
                Robot.shooter.setShooterMotor(Robot.shooter.shooterSpeedAdjust(Robot.limelight.getDistance()));
                if(Timer.getFPGATimestamp() - timeCheck > 1.5){Robot.autoSection = AutoSection.OPENING_ACTION;} 
                break;     
            case OPENING_ACTION :
                if (Robot.desiredMode == DesiredMode.BACK_UP_RED || Robot.desiredMode == DesiredMode.BACK_UP_BLUE ||
                    Robot.desiredMode == DesiredMode.ONE_BALL_RED || Robot.desiredMode == DesiredMode.ONE_BALL_BLUE){
                    limelightShoot();
                } else if (Robot.desiredMode == DesiredMode.TWO_BALL_RED || Robot.desiredMode == DesiredMode.TWO_BALL_BLUE){
                    Robot.intake.setHorizontalIntake(Constants.HORIZONTAL_INTAKE_SPEED);
                } else if (Robot.desiredMode == DesiredMode.THREE_BALL_RED || Robot.desiredMode == DesiredMode.THREE_BALL_BLUE) {
                    limelightShoot();
                    Robot.intake.setHorizontalIntake(Constants.HORIZONTAL_INTAKE_SPEED);
                }
                Robot.ledStrip.stripeRB();
                runRamsete().schedule();
                Robot.autoSection = AutoSection.CLOSING_ACTION;
                timeCheck = Timer.getFPGATimestamp();
                break;
            case CLOSING_ACTION :
                if (Robot.desiredMode == DesiredMode.BACK_UP_RED || Robot.desiredMode == DesiredMode.BACK_UP_BLUE){
                    Robot.autoSection = AutoSection.FINISH;
                } else if (Robot.desiredMode == DesiredMode.ONE_BALL_RED || Robot.desiredMode == DesiredMode.ONE_BALL_BLUE){
                    if(Timer.getFPGATimestamp() - timeCheck > 3.5){
                        Robot.intake.setHorizontalIntake(-Constants.HORIZONTAL_INTAKE_SPEED);
                        Robot.autoSection = AutoSection.FINISH;
                    }
                } else if (Robot.desiredMode == DesiredMode.TWO_BALL_RED || Robot.desiredMode == DesiredMode.TWO_BALL_BLUE){
                    if(Timer.getFPGATimestamp() - timeCheck > 3){
                        Robot.intake.setHorizontalIntake(0);
                        limelightShoot();
                        Robot.autoSection = AutoSection.FINISH;
                    }
                } else if (Robot.desiredMode == DesiredMode.THREE_BALL_RED || Robot.desiredMode == DesiredMode.THREE_BALL_BLUE) {
                    if(Timer.getFPGATimestamp() - timeCheck > 4){
                        Robot.intake.setHorizontalIntake(0);
                        limelightShoot();
                        Robot.autoSection = AutoSection.FINISH;
                    }
                }
                break;
            case FINISH :
                Robot.ledStrip.rainbow();
                break;
            case EXIT_AUTO:
            break;
        }
    }
}
