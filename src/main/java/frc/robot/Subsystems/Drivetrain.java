package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.HDD;
import frc.robot.Commands.DrivetrainTOCom;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.SerialPort;
import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain extends SubsystemBase{
    private CANSparkMax motorLeft0;
    private CANSparkMax motorLeft1;
    private CANSparkMax motorRight0;
    private CANSparkMax motorRight1;
    public RelativeEncoder m_leftEncoder0;
    public RelativeEncoder m_rightEncoder0; 
    public RelativeEncoder m_leftEncoder1;
    public RelativeEncoder m_rightEncoder1; 

    public final MecanumDrive m_drive;

    public AHRS gyro = new AHRS(SerialPort.Port.kUSB);

    public MecanumDriveOdometry odometry;
    
    public double initPose = 0.0;

    public Drivetrain (int l0, int l1, int r0, int r1){
        motorLeft0 = new CANSparkMax(l0, MotorType.kBrushless);
        motorLeft1 = new CANSparkMax(l1, MotorType.kBrushless);
        motorRight0 = new CANSparkMax(r0, MotorType.kBrushless);
        motorRight1 = new CANSparkMax(r1, MotorType.kBrushless);

        m_leftEncoder0 = motorLeft0.getEncoder();
        m_rightEncoder0 = motorRight0.getEncoder();
        m_leftEncoder1 = motorLeft1.getEncoder();
        m_rightEncoder1 = motorRight1.getEncoder();

        motorRight1.setInverted(true);
        motorRight0.setInverted(true);

        m_drive = new MecanumDrive(motorLeft0, motorLeft1, motorRight0, motorRight1);

        resetEncoders();

        odometry = new MecanumDriveOdometry(DriveConstants.kinematics, Rotation2d.fromDegrees(getHeading()));
        m_leftEncoder0.setPositionConversionFactor(4.1/42);
        m_rightEncoder0.setPositionConversionFactor(4.1/42);
        m_leftEncoder1.setPositionConversionFactor(4.1/42);
        m_rightEncoder1.setPositionConversionFactor(4.1/42);
    }

    //Every scheduler cycle, we pass our XBox controls so we can control the drivetrain and update its pose in the dashboards
    @Override
    public void periodic(){
        // Update the odometry in the periodic block
        odometry.update(
            gyro.getRotation2d(),
            new MecanumDriveWheelSpeeds(
                m_leftEncoder0.getVelocity(),
                m_leftEncoder1.getVelocity(),
                m_rightEncoder0.getVelocity(),
                m_rightEncoder1.getVelocity()));
        HDD.m_field.setRobotPose(odometry.getPoseMeters());

        setDefaultCommand(new DrivetrainTOCom());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds(){
        return new MecanumDriveWheelSpeeds(
            m_leftEncoder0.getVelocity(), 
            m_leftEncoder1.getVelocity(), 
            m_rightEncoder0.getVelocity(), 
            m_rightEncoder1.getVelocity()
        ); 
    }

    /**
     * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
     * speeds have no effect on the angular speed.
     *
     * @param xSpeed Speed of the robot in the x direction (forward/backwards).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
        m_drive.driveCartesian(ySpeed, xSpeed, rot, -gyro.getAngle());
        } else {
        m_drive.driveCartesian(ySpeed, xSpeed, rot);
        }
    }

    /** Sets the front left drive MotorController to a voltage. */
    public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
        motorLeft0.setVoltage(volts.frontLeftVoltage);
        motorRight0.setVoltage(volts.rearLeftVoltage);
        motorLeft1.setVoltage(volts.frontRightVoltage);
        motorRight1.setVoltage(volts.rearRightVoltage);
    }

    public void resetEncoders() {
        m_leftEncoder0.setPosition(0);
        m_rightEncoder0.setPosition(0);
        m_leftEncoder1.setPosition(0);
        m_rightEncoder1.setPosition(0);
    }

    /**
     * Gets the front left drive encoder.
     *
     * @return the front left drive encoder
     */
    public RelativeEncoder getFrontLeftEncoder() {
        return m_leftEncoder0;
    }

    /**
     * Gets the rear left drive encoder.
     *
     * @return the rear left drive encoder
     */
    public RelativeEncoder getRearLeftEncoder() {
        return m_leftEncoder1;
    }

    /**
     * Gets the front right drive encoder.
     *
     * @return the front right drive encoder
     */
    public RelativeEncoder getFrontRightEncoder() {
        return m_rightEncoder0;
    }

    /**
     * Gets the rear right drive encoder.
     *
     * @return the rear right encoder
     */
    public RelativeEncoder getRearRightEncoder() {
        return m_rightEncoder1;
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading(){
        // get the property
        return -gyro.getAngle();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }
}