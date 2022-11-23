package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Autonomous.Modes.AutoRoutineExample;

public class HDD {    
    public static SendableChooser<SequentialCommandGroup> m_chooser = new SendableChooser<>();

    //Field display to Shuffleboard
    public static Field2d m_field;
    public static Field2d logo;

    //Auto Commands
    public static AutoRoutineExample example = new AutoRoutineExample();

    public static SequentialCommandGroup desiredMode;
    public static SequentialCommandGroup prevMode;
  
    public static double x = 0.0;
    public static double y = 0.0;
    public static double angle = 0.0;

    public static void initBot(){

        m_chooser.setDefaultOption("Example", example);
        //m_chooser.addOption("Additional Mode", DesiredMode.ADDITIONAL_MODE);

        // Put the choosers on the dashboard
        SmartDashboard.putData(m_chooser);
        SmartDashboard.putNumber("Custom X",4);
        SmartDashboard.putNumber("Custom Y",4.1);
        SmartDashboard.putNumber("Custom Angle",0.0);
        SmartDashboard.putNumber("Startup Time",1.5);
        
        // Create and push Field2d to SmartDashboard.
        m_field = new Field2d();

        SmartDashboard.putData(m_field);
        LiveWindow.disableAllTelemetry();
        LiveWindow.enableTelemetry(Robot.drivetrain.gyro);
    }

    public static void updateStartupConfig(){
        desiredMode = m_chooser.getSelected();
    }
}