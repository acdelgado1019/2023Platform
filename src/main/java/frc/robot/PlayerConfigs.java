package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PlayerConfigs {
    public static enum Driver {
        CHRISTIAN,
        ANTHONY,
        ANTONIO
    }

    public static enum CoDriver {
        ANTHONY,
        CHRISTIAN,
        ANTONIO
    }

    public static SendableChooser<Driver> D_chooser = new SendableChooser<>();
    public static SendableChooser<CoDriver> CD_chooser = new SendableChooser<>();
    public static Driver driver;
    public static CoDriver coDriver;

    //drivetrain
    public static double accelerator;
    public static double steering;
    public static boolean autoTarget;
    public static double turnSpeed;
    public static double driveSpeed;

    //lift
    public static boolean intakeLiftMid;
    public static boolean intakeLiftHi;

    //horizontal
    public static boolean acceptIntake;
    public static boolean rejectIntake;
    
    //vertical
    public static boolean fireTrigger;
    public static boolean rejectTrigger;

    //shooter
    public static boolean lowPowerShooter;
    public static boolean midPowerShooter;
    public static boolean highPowerShooter;

    //limelight
    public static boolean switchPipeline;

    //climbers
    public static double climberLeftExtension;
    public static double climberRightExtension;
    public static boolean autoClimbTriggerA;
    public static boolean autoClimbTriggerB;
    public static boolean climberExtend;
    public static boolean climberRetract;
    public static boolean climberRotate;
    public static boolean climbModeReset;

    public static void initTeamSetup(){
        D_chooser.setDefaultOption("CHRISTIAN", Driver.CHRISTIAN);
        D_chooser.addOption("ANTHONY", Driver.ANTHONY);
        D_chooser.addOption("ANTONIO", Driver.ANTONIO);

        CD_chooser.setDefaultOption("ANTHONY", CoDriver.ANTHONY);
        CD_chooser.addOption("CHRISTIAN", CoDriver.CHRISTIAN);
        CD_chooser.addOption("ANTONIO", CoDriver.ANTONIO);

        SmartDashboard.putData(D_chooser);
        SmartDashboard.putData(CD_chooser);
    }

    public static void getPlayers(){
        driver = D_chooser.getSelected();
        coDriver = CD_chooser.getSelected();
    }

    public static void getDriverConfig(){
        switch(driver){
            case CHRISTIAN :
                //drivetrain
                accelerator = Robot.controller0.getJoystickAxis(Constants.LEFT_STICK_Y);
                steering = Robot.controller0.getJoystickAxis(Constants.RIGHT_STICK_X);
                autoTarget = Robot.controller0.getButton(Constants.BUTTON_A);
                turnSpeed = 0.4;
                driveSpeed = 1.0;

                //lift
                intakeLiftMid = Robot.controller0.getButton(Constants.RIGHT_BUMPER);
                intakeLiftHi = Robot.controller0.getButton(Constants.RIGHT_TRIGGER);

                //vertical
                fireTrigger = Robot.controller0.getButton(Constants.LEFT_BUMPER);
                rejectTrigger = Robot.controller0.getButton(Constants.LEFT_TRIGGER);

                //limelight
                switchPipeline = Robot.controller0.getButton(Constants.BUTTON_START);
            
            case ANTHONY :
                //drivetrain
                accelerator = Robot.controller0.getJoystickAxis(Constants.LEFT_STICK_Y);
                steering = Robot.controller0.getJoystickAxis(Constants.RIGHT_STICK_X);
                autoTarget = Robot.controller0.getButton(Constants.BUTTON_A);
                turnSpeed = 0.4;
                driveSpeed = 1.0;

                //lift
                intakeLiftMid = Robot.controller0.getButton(Constants.RIGHT_BUMPER);
                intakeLiftHi = Robot.controller0.getButton(Constants.RIGHT_TRIGGER);

                //vertical
                fireTrigger = Robot.controller0.getButton(Constants.LEFT_BUMPER);
                rejectTrigger = Robot.controller0.getButton(Constants.LEFT_TRIGGER);

                //limelight
                switchPipeline = Robot.controller0.getButton(Constants.BUTTON_START);

            case ANTONIO :
                //drivetrain
                accelerator = Robot.controller0.getJoystickAxis(Constants.LEFT_STICK_Y);
                steering = Robot.controller0.getJoystickAxis(Constants.RIGHT_STICK_X);
                autoTarget = Robot.controller0.getButton(Constants.BUTTON_A);
                turnSpeed = 0.4;
                driveSpeed = 1.0;

                //lift
                intakeLiftMid = Robot.controller0.getButton(Constants.RIGHT_BUMPER);
                intakeLiftHi = Robot.controller0.getButton(Constants.RIGHT_TRIGGER);

                //vertical
                fireTrigger = Robot.controller0.getButton(Constants.LEFT_BUMPER);
                rejectTrigger = Robot.controller0.getButton(Constants.LEFT_TRIGGER);

                //limelight
                switchPipeline = Robot.controller0.getButton(Constants.BUTTON_START);
        }
    }

    public static void getCoDriverConfig(){
        switch(coDriver){
            case ANTHONY :
                //horizontal
                acceptIntake = Robot.controller1.getButton(Constants.LEFT_BUMPER);
                rejectIntake = Robot.controller1.getButton(Constants.LEFT_TRIGGER);

                //shooter
                lowPowerShooter = Robot.controller1.getButton(Constants.BUTTON_A);
                midPowerShooter = Robot.controller1.getButton(Constants.BUTTON_X);
                highPowerShooter = Robot.controller1.getButton(Constants.BUTTON_Y);
                
                //climber
                climberLeftExtension = Robot.controller1.getJoystickAxis(Constants.LEFT_STICK_Y);
                climberRightExtension = Robot.controller1.getJoystickAxis(Constants.RIGHT_STICK_Y);
                autoClimbTriggerA = Robot.controller1.getButton(Constants.LEFT_JOYSTICK_BUTTON);
                autoClimbTriggerB = Robot.controller1.getButton(Constants.RIGHT_JOYSTICK_BUTTON);
                climberExtend = Robot.controller1.getButton(Constants.RIGHT_BUMPER);
                climberRetract = Robot.controller1.getButton(Constants.RIGHT_TRIGGER);
                climberRotate = Robot.controller1.getButton(Constants.BUTTON_B);
                climbModeReset = Robot.controller1.getButton(Constants.BUTTON_BACK);

            case CHRISTIAN :
                //horizontal
                acceptIntake = Robot.controller1.getButton(Constants.LEFT_BUMPER);
                rejectIntake = Robot.controller1.getButton(Constants.LEFT_TRIGGER);

                //shooter
                lowPowerShooter = Robot.controller1.getButton(Constants.BUTTON_A);
                midPowerShooter = Robot.controller1.getButton(Constants.BUTTON_X);
                highPowerShooter = Robot.controller1.getButton(Constants.BUTTON_Y);

                //climber
                climberLeftExtension = Robot.controller1.getJoystickAxis(Constants.LEFT_STICK_Y);
                climberRightExtension = Robot.controller1.getJoystickAxis(Constants.RIGHT_STICK_Y);
                autoClimbTriggerA = Robot.controller1.getButton(Constants.LEFT_JOYSTICK_BUTTON);
                autoClimbTriggerB = Robot.controller1.getButton(Constants.RIGHT_JOYSTICK_BUTTON);
                climberExtend = Robot.controller1.getButton(Constants.RIGHT_BUMPER);
                climberRetract = Robot.controller1.getButton(Constants.RIGHT_TRIGGER);
                climberRotate = Robot.controller1.getButton(Constants.BUTTON_B);
                climbModeReset = Robot.controller1.getButton(Constants.BUTTON_BACK);

            case ANTONIO :
                //horizontal
                acceptIntake = Robot.controller1.getButton(Constants.LEFT_BUMPER);
                rejectIntake = Robot.controller1.getButton(Constants.LEFT_TRIGGER);

                //shooter
                lowPowerShooter = Robot.controller1.getButton(Constants.BUTTON_A);
                midPowerShooter = Robot.controller1.getButton(Constants.BUTTON_X);
                highPowerShooter = Robot.controller1.getButton(Constants.BUTTON_Y);

                //climber
                climberLeftExtension = Robot.controller1.getJoystickAxis(Constants.LEFT_STICK_Y);
                climberRightExtension = Robot.controller1.getJoystickAxis(Constants.RIGHT_STICK_Y);
                autoClimbTriggerA = Robot.controller1.getButton(Constants.LEFT_JOYSTICK_BUTTON);
                autoClimbTriggerB = Robot.controller1.getButton(Constants.RIGHT_JOYSTICK_BUTTON);
                climberExtend = Robot.controller1.getButton(Constants.RIGHT_BUMPER);
                climberRetract = Robot.controller1.getButton(Constants.RIGHT_TRIGGER);
                climberRotate = Robot.controller1.getButton(Constants.BUTTON_B);
                climbModeReset = Robot.controller1.getButton(Constants.BUTTON_BACK);
        }
    }
}
