package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HDD;
import frc.robot.Commands.DrivetrainTOCom;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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

    private Translation2d frontLeftMeters = new Translation2d(Constants.kTrackwidthMeters, Rotation2d.fromDegrees(135));
    private Translation2d backLeftMeters = new Translation2d(Constants.kTrackwidthMeters, Rotation2d.fromDegrees(-135));
    private Translation2d frontRightMeters = new Translation2d(Constants.kTrackwidthMeters, Rotation2d.fromDegrees(45));
    private Translation2d backRightMeters = new Translation2d(Constants.kTrackwidthMeters, Rotation2d.fromDegrees(-45));
    public MecanumDriveKinematics kinematics = new MecanumDriveKinematics(frontLeftMeters, backLeftMeters, frontRightMeters, backRightMeters);
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

        odometry = new MecanumDriveOdometry(kinematics, Rotation2d.fromDegrees(getHeading()));
        m_leftEncoder0.setPositionConversionFactor(4.1/42);
        m_rightEncoder0.setPositionConversionFactor(4.1/42);
        m_leftEncoder1.setPositionConversionFactor(4.1/42);
        m_rightEncoder1.setPositionConversionFactor(4.1/42);
    }

    //Every scheduler cycle, we pass our XBox controls so we can control the drivetrain and update its pose in the dashboards
    @Override
    public void periodic(){
        odometry.update(gyro.getRotation2d(),
                    getWheelSpeeds());
        HDD.m_field.setRobotPose(odometry.getPoseMeters());

        setDefaultCommand(new DrivetrainTOCom());
    }

    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds(){
        return new MecanumDriveWheelSpeeds(m_leftEncoder0.getVelocity(), m_leftEncoder1.getVelocity(), m_rightEncoder0.getVelocity(), m_rightEncoder1.getVelocity()); 
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public void driveVolts(double fLeftVolts, double bLeftVolts, double fRightVolts, double bRightVolts){
        var batteryVoltage = RobotController.getBatteryVoltage();
        if (Math.max(
                Math.max(Math.abs(fLeftVolts), Math.abs(bLeftVolts)), 
                Math.max(Math.abs(fRightVolts), Math.abs(bRightVolts))
            ) > batteryVoltage) {
            fLeftVolts *= batteryVoltage / 12.0;
            bLeftVolts *= batteryVoltage / 12.0;
            fRightVolts *= batteryVoltage / 12.0;
            bRightVolts *= batteryVoltage / 12.0;
        }

        motorLeft0.setVoltage(fLeftVolts);
        motorLeft1.setVoltage(bLeftVolts);
        motorRight0.setVoltage(fRightVolts);
        motorRight1.setVoltage(bRightVolts);
        m_drive.feed();
    }

    public void resetEncoders() {
        m_leftEncoder0.setPosition(0);
        m_rightEncoder0.setPosition(0);
        m_leftEncoder1.setPosition(0);
        m_rightEncoder1.setPosition(0);
    }

    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }

    public double getHeading(){
        // get the property
        return -gyro.getAngle();
    }
}