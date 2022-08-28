// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.AutoMethods;
import frc.robot.Subsystems.Climbers;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Climbers.AutoClimbStep;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.LEDs;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {  
  //Subsystem Declarations

  public static final Drivetrain drivetrain = new Drivetrain(
    Constants.LEFT_DRIVE_TRAIN_0,
    Constants.LEFT_DRIVE_TRAIN_1,
    Constants.RIGHT_DRIVE_TRAIN_0,
    Constants.RIGHT_DRIVE_TRAIN_1
  );
  
  public static final Intake intake = new Intake(
    Constants.HORIZONTAL_INTAKE,
    Constants.TRIGGER,
    Constants.INTAKE_LIFT
  );

  public static final Shooter shooter = new Shooter(
    Constants.SHOOTER
  );

  public static final Climbers climbers = new Climbers(
    Constants.LEFT_CLIMBER_0,
    Constants.LEFT_CLIMBER_1,
    Constants.LEFT_CLIMBER_ROTATE, 
    Constants.RIGHT_CLIMBER_0, 
    Constants.RIGHT_CLIMBER_1, 
    Constants.RIGHT_CLIMBER_ROTATE
  );
  
  public static final Limelight limelight = new Limelight();

  public static final LEDs ledStrip = new LEDs(4,10);

  //Controllers
  public static final Controller controller0 = new Controller(Constants.DRIVER_CONTROLLER_0);
  public static final Controller controller1 = new Controller(Constants.DRIVER_CONTROLLER_1);

  //Auto Commands
  public static enum DesiredMode {
    BACK_UP_BLUE,
    BACK_UP_RED, 
    ONE_BALL_RED,
    ONE_BALL_BLUE,
    TWO_BALL_RED,
    TWO_BALL_BLUE,
    THREE_BALL_RED,
    THREE_BALL_BLUE,
  }

  public enum AutoSection {
    STARTUP,
    OPENING_ACTION,
    CLOSING_ACTION,
    FINISH,
    EXIT_AUTO
  }

  public static DesiredMode desiredMode;
  public static DesiredMode prevMode;
  public static AutoSection autoSection;
  public static String prevAuto = "";
  public double timeCheck;
  
  public static SendableChooser<DesiredMode> m_chooser = new SendableChooser<>();

  //Field display to Shuffleboard
  public static Field2d m_field;
  public static Field2d logo;

  //Test Timer & Flag
  Timer timer = new Timer();

  /*
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("One Ball Red", DesiredMode.ONE_BALL_RED);
    m_chooser.addOption("Two Ball Red", DesiredMode.TWO_BALL_RED);
    m_chooser.addOption("Three Ball Red", DesiredMode.THREE_BALL_RED);
    m_chooser.addOption("Back Up Red", DesiredMode.BACK_UP_RED);
    m_chooser.addOption("One Ball Blue", DesiredMode.ONE_BALL_BLUE);
    m_chooser.addOption("Two Ball Blue", DesiredMode.TWO_BALL_BLUE);
    m_chooser.addOption("Three Ball Blue", DesiredMode.THREE_BALL_BLUE);
    m_chooser.addOption("Back Up Blue", DesiredMode.BACK_UP_BLUE);

    // Put the choosers on the dashboard
    SmartDashboard.putData(m_chooser);

    AutoMethods.getConstraint();
    AutoMethods.getTrajectoryConfig();
      
    // Create and push Field2d to SmartDashboard.
    m_field = new Field2d();

    SmartDashboard.putData(m_field);
    LiveWindow.disableAllTelemetry();
    LiveWindow.enableTelemetry(drivetrain.m_gyro);

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Left Rotator Sim", climbers.L_mech2d);
    climbers.L_RotatorTower.setColor(new Color8Bit(Color.kGray));

    SmartDashboard.putData("Right Rotator Sim", climbers.R_mech2d);
    climbers.R_RotatorTower.setColor(new Color8Bit(Color.kDarkGray));

    SmartDashboard.putData("Intake Lift Sim", intake.intake_mech2d);
    intake.intakeTower.setColor(new Color8Bit(Color.kFirstRed));

    climbers.autoClimbStep = AutoClimbStep.MANUAL_MODE;
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_field.setRobotPose(drivetrain.odometry.getPoseMeters());
    drivetrain.m_drive.feed();
  }

  @Override
  public void autonomousInit() {
    Constants.teamColor = DriverStation.getAlliance().toString();
    climbers.resetEncoders();
    intake.resetEncoder();
    ledStrip.stripeRB();
    autoSection = AutoSection.STARTUP;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    AutoMethods.runAutonomous(timeCheck);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    autoSection = AutoSection.EXIT_AUTO;
    Constants.teamColor = DriverStation.getAlliance().toString();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    intake.setIntakeLift(0.0);
    climbers.setLeftClimberRotation(0.0);
    climbers.setRightClimberRotation(0.0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    desiredMode = m_chooser.getSelected();
    
    if (prevMode!=desiredMode){
      AutoMethods.getTrajectory();
      m_field.getObject("traj").setTrajectory(AutoMethods.trajectory);
      AutoMethods.resetOdometry(AutoMethods.trajectory);
      prevMode = desiredMode;
    }

    ledStrip.mardiGras();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    climbers.resetEncoders();
    intake.resetEncoder();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
