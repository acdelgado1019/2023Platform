package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.Autonomous.AutoMethods;

public class HDD {    
    public static SendableChooser<DesiredMode> m_chooser = new SendableChooser<>();

    //Field display to Shuffleboard
    public static Field2d m_field;
    public static Field2d logo;

    //Auto Commands
    public static enum DesiredMode {
        BACK_UP,
        ONE_BALL_RED,
        ONE_BALL_BLUE,
        TWO_BALL_RED,
        TWO_BALL_BLUE,
        THREE_BALL_RED,
        THREE_BALL_BLUE,
    }

    public static DesiredMode desiredMode;
    public static DesiredMode prevMode;
  
    public static double x = 0.0;
    public static double y = 0.0;
    public static double angle = 0.0;

    public static void initBot(){

        m_chooser.setDefaultOption("One Ball Red", DesiredMode.ONE_BALL_RED);
        m_chooser.addOption("Two Ball Red", DesiredMode.TWO_BALL_RED);
        m_chooser.addOption("Three Ball Red", DesiredMode.THREE_BALL_RED);
        m_chooser.addOption("One Ball Blue", DesiredMode.ONE_BALL_BLUE);
        m_chooser.addOption("Two Ball Blue", DesiredMode.TWO_BALL_BLUE);
        m_chooser.addOption("Three Ball Blue", DesiredMode.THREE_BALL_BLUE);
        m_chooser.addOption("Back Up", DesiredMode.BACK_UP);

        // Put the choosers on the dashboard
        SmartDashboard.putData(m_chooser);
        SmartDashboard.putNumber("Custom X",4);
        SmartDashboard.putNumber("Custom Y",4.1);
        SmartDashboard.putNumber("Custom Angle",0.0);

        AutoMethods.getConstraint();
        AutoMethods.getTrajectoryConfig();
        
        // Create and push Field2d to SmartDashboard.
        m_field = new Field2d();

        SmartDashboard.putData(m_field);
        LiveWindow.disableAllTelemetry();
        LiveWindow.enableTelemetry(Robot.drivetrain.m_gyro);
    }

    public static void updateStartupConfig(){
        desiredMode = m_chooser.getSelected();
    
        if (desiredMode == DesiredMode.BACK_UP){
            x = SmartDashboard.getNumber("Custom X", 4);
            y = SmartDashboard.getNumber("Custom Y", 4.1);
            angle = SmartDashboard.getNumber("Custom Angle", 0.0)-90;
        }

        if (prevMode!=desiredMode || desiredMode == DesiredMode.BACK_UP){
        AutoMethods.getTrajectory();
        m_field.getObject("traj").setTrajectory(AutoMethods.trajectory);
        AutoMethods.resetOdometry(AutoMethods.trajectory);
        prevMode = desiredMode;
        } 
    }
}