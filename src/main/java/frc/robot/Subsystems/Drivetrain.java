package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Commands.DrivetrainTOCom;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain extends SubsystemBase{
    private CANSparkMax motorLeft0;
    private CANSparkMax motorLeft1;
    private CANSparkMax motorRight0;
    private CANSparkMax motorRight1;
    private Encoder m_leftEncoder = new Encoder(0, 1);
    private Encoder m_rightEncoder = new Encoder(2, 3); 
    private EncoderSim m_leftEncoderSim = new EncoderSim(m_leftEncoder);
    private EncoderSim m_rightEncoderSim = new EncoderSim(m_rightEncoder);  
    private final MotorControllerGroup m_leftMotors;
    private final MotorControllerGroup m_rightMotors;
    public final DifferentialDrive m_drive;
    public DifferentialDrivetrainSim m_driveSim;

    public AnalogGyro m_gyro = new AnalogGyro(1);
    public AnalogGyroSim gyro = new AnalogGyroSim(m_gyro);

    private Field2d m_field = new Field2d();

    public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);
    public DifferentialDriveOdometry odometry;

    public double initPose = 0.0;

    public Drivetrain (int l0, int l1, int r0, int r1){
        motorLeft0 = new CANSparkMax(l0, MotorType.kBrushless);
        motorLeft1 = new CANSparkMax(l1, MotorType.kBrushless);
        motorRight0 = new CANSparkMax(r0, MotorType.kBrushless);
        motorRight1 = new CANSparkMax(r1, MotorType.kBrushless);

        m_leftMotors = new MotorControllerGroup(motorLeft0,motorLeft1);
        m_rightMotors = new MotorControllerGroup(motorRight0,motorRight1); 

        m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

        m_driveSim = new DifferentialDrivetrainSim(
            DCMotor.getNEO(2),       // 2 NEO motors on each side of the drivetrain.
            6,                    // 6:1 gearing reduction.
            1,                     // MOI of 4.8 kg m^2 (need to obtain from CAD model).
            47.6,                    // The mass of the robot is 105 lbs -> 47.6 kg.
            Units.inchesToMeters(3), // The robot uses 3" radius wheels.
            Constants.kTrackwidthMeters, //Distance between wheels

            // The standard deviations for measurement noise:
            // x and y:          0.001 m
            // heading:          0.001 rad
            // l and r velocity: 0.1   m/s
            // l and r position: 0.005 m
            null//VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        );

        m_rightMotors.setInverted(true);

        resetEncoders();

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
        m_leftEncoder.setDistancePerPulse(4.1/42);
        m_rightEncoder.setDistancePerPulse(4.1/42);
        SmartDashboard.putData("Field", m_field);
    }

    //Every scheduler cycle, we pass our XBox controls so we can control the drivetrain and update its pose in the dashboards
    @Override
    public void periodic(){
        odometry.update(m_gyro.getRotation2d(),
                    m_leftEncoder.getDistance(),
                    m_rightEncoder.getDistance());
        m_field.setRobotPose(odometry.getPoseMeters());

        setDefaultCommand(new DrivetrainTOCom());
    }

    @Override
    public void simulationPeriodic(){
          // Set the inputs to the system. Note that we need to convert
        // the [-1, 1] PWM signal to voltage by multiplying it by the
        // robot controller voltage.
        m_driveSim.setInputs(motorLeft1.get() * RobotController.getBatteryVoltage(),
        motorRight1.get() * RobotController.getBatteryVoltage());

        // Advance the model by 20 ms
        m_driveSim.update(0.02);

        // Update all of our sensors.
        m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
        m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
        m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
        m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
        gyro.setAngle(-m_driveSim.getHeading().getDegrees());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate()); 
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void tankDriveVolts(double leftVolts, double rightVolts){
        var batteryVoltage = RobotController.getBatteryVoltage();
        if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
            leftVolts *= batteryVoltage / 12.0;
            rightVolts *= batteryVoltage / 12.0;
        }
        m_leftMotors.setVoltage(leftVolts);
        m_rightMotors.setVoltage(-rightVolts);
        m_drive.feed();
    }

    public void resetEncoders() {
        m_leftEncoder.reset();
        m_rightEncoder.reset();
    }

    public double getAverageEncoderDistance() {
        return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    public double getHeading(){
        // get the property
        return -gyro.getAngle();
    }

    public double getNormHeading(){
        double heading = 0.0;
        if((getHeading()+ initPose) % 360 > -180 && (getHeading()+ initPose) % 360 < 180){
            heading = (getHeading()+ initPose)%360;
        } else if((getHeading()+ initPose) % 360 < -180){
            heading = 360 + (getHeading()+ initPose)%360;
        } else if((getHeading()+ initPose) % 360 > 180){
            heading = -360 + (getHeading()+ initPose)%360;
        }
        return heading;
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }

    //Drive Methods
    public void setLeftDrivetrain(double speed){
        motorLeft0.set(speed);
        motorLeft1.set(speed);
        m_drive.feed();
    }

    public void setRightDrivetrain(double speed){
        motorRight0.set(-speed);
        motorRight1.set(-speed);
        m_drive.feed();
    }

        //Adjusts the pose of the robot to center on the hub
        public void limelightTrack()
        {
            double degOff = 0.0;
            
            if(Robot.limelight.getTV() != 0){
                degOff = Robot.limelight.getTX();
            } else {
                degOff = getNormHeading() - Robot.limelight.getOffset();
            }
            if(Math.abs(degOff) > 1){
                    double speed = .15 * degOff/90;
                    setLeftDrivetrain(speed);
                    setRightDrivetrain(speed);
            }
        }

}