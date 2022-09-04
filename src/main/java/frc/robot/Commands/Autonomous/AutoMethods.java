package frc.robot.Commands.Autonomous;

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
import frc.robot.HDD;
import frc.robot.Robot;

public class AutoMethods {
    public static DifferentialDriveVoltageConstraint autoVoltageConstraint;
    public static TrajectoryConfig config;
    public static Trajectory trajectory;
    public static RamseteCommand ramseteCommand;

    private static double x1;
    private static double x2;
    private static double x3;
    private static double y1;
    private static double y2;
    private static double y3;


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

    public static void lineDrive(double distance){
        double error = distance - Robot.drivetrain.getAverageEncoderDistance();
        if(Math.abs(error) > 0.1){
                double speed = 0.2 * error;
                Robot.drivetrain.tankDriveVolts(speed*12, speed*12);
        }
    }

    public static void rotate(double angle){
        double degOff = Robot.drivetrain.getNormHeading() - angle;
        if (Math.abs(degOff)>180){degOff = -360+degOff;}
        if(Math.abs(degOff) > 1){
                double speed = Constants.ROTATE_KP * degOff;
                Robot.drivetrain.tankDriveVolts(speed*12, -speed*12);
        }
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
        switch (HDD.desiredMode){
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
            case FOUR_BALL_BLUE:
                Robot.drivetrain.initPose = -90;
                trajectory =
                TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing goal
                new Pose2d(7.6, 0.7, new Rotation2d(Units.degreesToRadians(90))),
                // Pass through ball 2 and 3
                List.of(new Translation2d(7.6, 1.85), new Translation2d(5.0, 1.85)),
                // Turn to goal and come in range
                new Pose2d(1.4, 1.4, new Rotation2d(-140*Math.PI*2/360)),
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
            
            case FOUR_BALL_RED:
                Robot.drivetrain.initPose = 90;
                trajectory =
                TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing goal
                new Pose2d(8.8, 7.5, new Rotation2d(Units.degreesToRadians(-90))),
                // Pass through ball 2 and 3
                List.of(new Translation2d(8.8, 6.35), new Translation2d(11.4, 6.35)),
                // Turn to goal and come in range
                new Pose2d(14.8, 6.8, new Rotation2d(40*Math.PI*2/360)),
                // Pass config
                config);
                break;
            case BACK_UP :  
                Robot.drivetrain.initPose = HDD.angle+90;
                config.setReversed(true);

                x1 = HDD.x+Math.sin(Units.degreesToRadians(HDD.angle));
                y1 = HDD.y-Math.cos(Units.degreesToRadians(HDD.angle));
                x2 = HDD.x+2*Math.sin(Units.degreesToRadians(HDD.angle));
                y2 = Math.max(Math.min(HDD.y-2*Math.cos(Units.degreesToRadians(HDD.angle)),8.2),0.2);
                x3 = HDD.x+3*Math.sin(Units.degreesToRadians(HDD.angle));
                y3 = Math.max(Math.min(HDD.y-3*Math.cos(Units.degreesToRadians(HDD.angle)),8.2),0);

                trajectory =
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(HDD.x, HDD.y, new Rotation2d(Units.degreesToRadians(HDD.angle+90))),
                List.of(
                    new Translation2d(x1, y1), 
                    new Translation2d(x2, y2)
                ), 
                new Pose2d(x3, y3, new Rotation2d(Units.degreesToRadians(HDD.angle+90))),
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
        trajectory = getTrajectory();
        ramseteCommand = getRamsete();
        resetOdometry(trajectory);
        return ramseteCommand.andThen(() -> Robot.drivetrain.tankDriveVolts(0, 0));
    }
}
