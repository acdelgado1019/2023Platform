// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
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
import frc.robot.Subsystems.Intake;

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
  
  public static final Intake shooterIntake = new Intake(
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

  //Controllers
  public static final Controller controller0 = new Controller(Constants.DRIVER_CONTROLLER_0);
  public static final Controller controller1 = new Controller(Constants.DRIVER_CONTROLLER_1);

  //Auto Commands
  public static String autoSequence;
  public static String prevAuto = "";
  public static String team;
  public static int path = 0;
  private final String oneBall = "One Ball Auto";
  private final String twoBall = "Two Ball Auto";
  private final String threeBall = "Three Ball Auto";
  private final String slalom = "Slalom";
  private final String barrel = "Barrel";
  public Boolean preMoveMode;
  public Boolean moveMode;
  public Boolean postMoveMode;
  public double timeCheck;
  
  public static SendableChooser<String> m_chooser = new SendableChooser<>();
  public static SendableChooser<String> t_chooser = new SendableChooser<>();

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
    m_chooser.setDefaultOption("Three Ball Auto", threeBall);
    m_chooser.addOption("Two Ball Auto", twoBall);
    m_chooser.addOption("One Ball Auto", oneBall);
    m_chooser.addOption("Slalom", slalom);
    m_chooser.addOption("Barrel", barrel);

    t_chooser.setDefaultOption("Red", "RED");
    t_chooser.addOption("Blue", "BLUE");

    // Put the choosers on the dashboard
    SmartDashboard.putData(m_chooser);
    SmartDashboard.putData(t_chooser);
    SmartDashboard.putString("Auto Step", "NOT STARTED");

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

    SmartDashboard.putData("Intake Lift Sim", shooterIntake.intake_mech2d);
    shooterIntake.intakeTower.setColor(new Color8Bit(Color.kFirstRed));
    
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
    climbers.resetEncoders();
    shooterIntake.resetEncoder();
    preMoveMode = true;
    moveMode = false;
    postMoveMode = false;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (preMoveMode){
      if (autoSequence == "One Ball Auto"){
        AutoMethods.timerDrive(0.6, 2);
        SmartDashboard.putString("Auto Step", "Delay");
        Timer.delay(0);

        SmartDashboard.putString("Auto Step", "Intake Down");
        AutoMethods.lowerIntake();
        SmartDashboard.putString("Auto Step", "Shooting");
        AutoMethods.limelightShoot(Constants.SHOOTER_HI_SPEED);
        SmartDashboard.putString("Auto Step", "Run Away");
      } else if (autoSequence == "Two Ball Auto"){
        SmartDashboard.putString("Auto Step", "Intake Down");
        AutoMethods.lowerIntake();
        SmartDashboard.putString("Auto Step", "Run Intake");
        AutoMethods.runIntake(Constants.HORIZONTAL_INTAKE_SPEED);
        SmartDashboard.putString("Auto Step", "Collect");
      } else if (autoSequence == "Three Ball Auto") {
        SmartDashboard.putString("Auto Step", "Intake Down");
        AutoMethods.lowerIntake();
        SmartDashboard.putString("Auto Step", "Shooting");
        AutoMethods.limelightShoot(Constants.SHOOTER_HI_SPEED);
        SmartDashboard.putString("Auto Step", "Run Intake");
        AutoMethods.runIntake(Constants.HORIZONTAL_INTAKE_SPEED);
        SmartDashboard.putString("Auto Step", "Collect");
      }
      moveMode = true;
      preMoveMode = false;
    } else if (moveMode){
      AutoMethods.runRamsete(path).schedule();
      postMoveMode = true;
      moveMode = false;
      timeCheck = Timer.getFPGATimestamp();
    } else if (postMoveMode){
      if(Timer.getFPGATimestamp() - timeCheck > 5){
        if (autoSequence == "One Ball Auto"){
          SmartDashboard.putString("Auto Step", "Run Intake");
          AutoMethods.runIntake(-Constants.HORIZONTAL_INTAKE_SPEED);
          SmartDashboard.putString("Auto Step", "Rotate");
          AutoMethods.rotate(1);
        } else if (autoSequence == "Two Ball Auto"){
          SmartDashboard.putString("Auto Step", "Stop Intake");
          AutoMethods.runIntake(0);
          SmartDashboard.putString("Auto Step", "Shoot");
          AutoMethods.limelightShoot(Constants.SHOOTER_LOW_SPEED);
        } else if (autoSequence == "Three Ball Auto") {
          SmartDashboard.putString("Auto Step", "Stop Intake");
          AutoMethods.runIntake(0);
          SmartDashboard.putString("Auto Step", "Shoot");
          AutoMethods.limelightShoot(Constants.SHOOTER_LOW_SPEED);
        }
        postMoveMode = false;
      }
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    shooterIntake.setIntakeLift(0.0);
    climbers.setLeftClimberRotation(0.0);
    climbers.setRightClimberRotation(0.0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    autoSequence = m_chooser.getSelected();
    team = Robot.t_chooser.getSelected();

    if (team == "RED" && autoSequence == "One Ball Auto"){path = 6;}
    else if (team == "BLUE" && autoSequence == "One Ball Auto"){path = 5;}
    else if (team == "RED" && autoSequence == "Three Ball Auto"){path = 4;}
    else if (team == "RED" && autoSequence == "Two Ball Auto"){path = 3;}
    else if (team == "BLUE" && autoSequence == "Three Ball Auto"){path = 2;}
    else if (team == "BLUE" && autoSequence == "Two Ball Auto"){path = 1;}
    else if (autoSequence == "Slalom"){path = 100;}
    else if (autoSequence == "Barrel"){path = 101;}

    AutoMethods.getTrajectory(path);
    m_field.getObject("traj").setTrajectory(AutoMethods.trajectory);
    AutoMethods.resetOdometry(AutoMethods.trajectory);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    climbers.resetEncoders();
    shooterIntake.resetEncoder();
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
