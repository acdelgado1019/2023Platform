package frc.robot;

public class PlayerConfigs {
    
    //drivetrain
    public static double accelerator;
    public static double steering;
    public static double xMovement;
    public static double yMovement;
    public static double turnMovement;
    public static double turnSpeed;
    public static double driveSpeed;

    //limelight
    public static boolean switchPipeline;

    public static void getDriverConfig(){
        //drivetrain
        accelerator = Robot.controller0.getRightY();
        steering = Robot.controller0.getRightX();
        xMovement = Robot.controller0.getLeftX();
        yMovement = Robot.controller0.getLeftY();
        turnMovement = Robot.controller0.getRightX();
        turnSpeed = 0.1;
        driveSpeed = 0.5;

        //limelight
        switchPipeline = Robot.controller0.getCircleButton();
    }

    public static void getCoDriverConfig(){  
    }

    
}
