package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    //Motor IDs
        //Drivetrain
        public static final int LEFT_DRIVE_TRAIN_0 = 19;
        public static final int LEFT_DRIVE_TRAIN_1 = 20;
        public static final int RIGHT_DRIVE_TRAIN_0 = 1;
        public static final int RIGHT_DRIVE_TRAIN_1 = 2;

        //Climbers
        public static final int LEFT_CLIMBER_0 = 9;
        public static final int LEFT_CLIMBER_1 = 12;
        public static final int LEFT_CLIMBER_ROTATE = 18;
        public static final int RIGHT_CLIMBER_0 = 10;
        public static final int RIGHT_CLIMBER_1 = 11;
        public static final int RIGHT_CLIMBER_ROTATE = 3;

        //Shooter
        public static final int SHOOTER = 15;

        //Intake
        public static final int TRIGGER = 6;
        public static final int HORIZONTAL_INTAKE = 17;
        public static final int INTAKE_LIFT = 5;

        //Dumper
        public static final int DUMPER_INTAKE = 17;
        public static final int LEFT_DUMPER_LIFT = 5;
        public static final int RIGHT_DUMPER_LIFT = 15;

    //Controller Assignments
        public static final int DRIVER_CONTROLLER_0 = 0;
        public static final int DRIVER_CONTROLLER_1 = 1;
        
        //Control Axis
        public static final int LEFT_STICK_X = 0;
        public static final int LEFT_STICK_Y = 1;
        public static final int RIGHT_STICK_X = 2;
        public static final int RIGHT_STICK_Y = 3;

        //Control D-Pad
        public static final int DPAD_X = 2;
        public static final int DPAD_Y = 3;

        //Control Buttons
        public static final int BUTTON_A = 2;
        public static final int BUTTON_B = 3;
        public static final int BUTTON_X = 1;
        public static final int BUTTON_Y = 4;
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;

        public static final int LEFT_TRIGGER = 7;
        public static final int RIGHT_TRIGGER = 8;

        public static final int BUTTON_BACK = 9;
        public static final int BUTTON_START = 10;
        public static final int LEFT_JOYSTICK_BUTTON = 11;
        public static final int RIGHT_JOYSTICK_BUTTON = 12;

        //FMS Data
        public static String teamColor;

    //Speed Variables
        public static final double MAX_DRIVE_SPEED = .8; // Min = 0, Max = 1
        // public static final double FULL_SPEED = 1;
        // public static final double SLOW_SPEED = .4;
        public static final double CLIMBER_ROTATION_SPEED = .25;
        public static final double CLIMBER_ROTATION_STATIC = -.02;
        public static final double MAX_TURN_SPEED = 0.15; // Min = 0, Max = 1;
        public static final double LEFT_RIGHT_TRIM = 0; // Min = -1, Max = 1;+
        public static final double INTAKE_LIFT_SPEED = .3;
        public static final double HORIZONTAL_INTAKE_SPEED = -.8;
        public static final double TRIGGER_SPEED = -1;
        public static final double SHOOTER_HI_SPEED = 12;
        public static final double SHOOTER_MID_SPEED = 10;
        public static final double SHOOTER_LOW_SPEED = 7.2;
        public static final double SHOOTER_IDLE_SPEED = 3;

    //Characterization Constants
        public static final double ksVolts = 0.63969;
        public static final double kvVoltSecondsPerMeter = 0.34464;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        public static final double kPDriveVel = 3.9966;
        public static final double kTrackwidthMeters = Units.inchesToMeters(19.25);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double kEncoderDistancePerPulse = 1/6 * 2 * Math.PI * Units.inchesToMeters(4);

        public static final double camHeight = 40;
        public static final double goalHeight = 102.5;
        public static final double camAngle = 15;

    //Climber Constants
        public static final double m_RotatorReduction = 30;
        public static final double m_RotatorMass = 5.4; // Kilograms
        public static final double m_RotatorLength = Units.inchesToMeters(42);
        public static final double kRotatorEncoderDistPerPulse = 2.0 * Math.PI / m_RotatorReduction;
        public static final double kClimberEncoderDistPerPulse = 2.0 * Math.PI / 8;

        // The P gain for the PID controller that drives this Rotator.
        public static double kLRotatorKp = 10;

        // The P gain for the PID controller that drives this Rotator.
        public static double kRRotatorKp = 10;

        public static double RotatorFullPositionDeg = 116.0;
        public static double RotatorUnhookPositionDeg = 93.0;

    //Intake Lift Constants
        public static final double m_IntakeLiftReduction = 125;
        public static final double m_IntakeLiftMass = 8; // Kilograms
        public static final double m_IntakeLiftLength = Units.inchesToMeters(40);
        public static final double kIntakeLiftEncoderDistPerPulse = 2.0 * Math.PI / 42/125;

        // The P gain for the PID controller that drives this Lift.
        public static double kIntakeLiftKp = 5;
        public static double hiILPositionDeg = 70;
        public static double midILPositionDeg = 30;
        public static double loILPositionDeg = 0;
}