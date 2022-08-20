package frc.robot.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ClimbersTOCom extends CommandBase{    

    public ClimbersTOCom(){
        addRequirements(Robot.climbers);
    }

    @Override
    public void execute(){    
        double controller1_leftJoystickY = Robot.controller1.getJoystickAxis(Constants.LEFT_STICK_Y);
        double controller1_rightJoystickY = Robot.controller1.getJoystickAxis(Constants.RIGHT_STICK_Y);

        boolean controller1_rightTrigger = Robot.controller1.getButton(Constants.RIGHT_TRIGGER);
        boolean controller1_rightBumper = Robot.controller1.getButton(Constants.RIGHT_BUMPER);

        double controller1_dpad = Robot.controller1.getPOV();

        if(Robot.controller1.getButton(Constants.BUTTON_BACK)){Robot.climbers.resetClimbMode();}

        if(controller1_rightTrigger){
            Robot.climbers.setClimbMode();
            Robot.climbers.setLeftClimber(1);
            Robot.climbers.setRightClimber(1);
        } else if (controller1_rightBumper) {
            Robot.climbers.setClimbMode();
            Robot.climbers.setLeftClimber(-1);
            Robot.climbers.setRightClimber(-1);
            Robot.climbers.setClimberRotation(Units.degreesToRadians(Constants.RotatorUnhookPositionDeg));
        } else {
            Robot.climbers.setLeftClimber(controller1_leftJoystickY);
            Robot.climbers.setRightClimber(controller1_rightJoystickY);
        }

        if (Robot.climbers.getClimbMode()){Robot.ledStrip.rainbow();}

        if (controller1_dpad == 180){
            Robot.climbers.setClimberRotation(Units.degreesToRadians(Constants.RotatorFullPositionDeg));
        } else if(!controller1_rightBumper){
            Robot.climbers.setClimberRotation(Units.degreesToRadians(90.0));
        }

        Robot.climbers.updateDashboard();
    }
}