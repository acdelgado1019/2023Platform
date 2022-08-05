package frc.robot.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeTOCom extends CommandBase{

    public IntakeTOCom(){
        addRequirements(Robot.shooterIntake);
    }

    @Override
    public void execute(){
        //lift
        boolean controller0_leftBumper = Robot.controller0.getButton(Constants.LEFT_BUMPER);
        boolean controller0_leftTrigger = Robot.controller0.getTrigger(Constants.LEFT_TRIGGER);

        //horizontal
        boolean controller0_rightBumper = Robot.controller0.getButton(Constants.RIGHT_BUMPER);
        boolean controller0_rightTrigger = Robot.controller0.getTrigger(Constants.RIGHT_TRIGGER);
        
        //vertical
        boolean controller1_rightBumper = Robot.controller1.getButton(Constants.RIGHT_BUMPER);
        boolean controller1_rightTrigger = Robot.controller1.getTrigger(Constants.RIGHT_TRIGGER);

        Robot.shooterIntake.setHorizontalIntake(controller0_leftTrigger ? Constants.HORIZONTAL_INTAKE_SPEED : (controller0_leftBumper ? -Constants.HORIZONTAL_INTAKE_SPEED : 0));
        
        if (controller0_rightBumper){
            var pidOutput =
                Robot.shooterIntake.Lift_controller.calculate(Robot.shooterIntake.getEncoder(), Units.degreesToRadians(Constants.hiILPositionDeg));
            Robot.shooterIntake.setIntakeLift(pidOutput);
        } else if(controller0_rightTrigger){
            var pidOutput =
                Robot.shooterIntake.Lift_controller.calculate(Robot.shooterIntake.getEncoder(), Units.degreesToRadians(Constants.midILPositionDeg));
            Robot.shooterIntake.setIntakeLift(pidOutput);
        } else{
            var pidOutput =
                Robot.shooterIntake.Lift_controller.calculate(Robot.shooterIntake.getEncoder(), Units.degreesToRadians(Constants.loILPositionDeg));
            Robot.shooterIntake.setIntakeLift(pidOutput);
        }

        if (controller1_rightBumper){
            Robot.shooterIntake.pulse();
        } else if (controller1_rightTrigger){
            Robot.shooterIntake.setTrigger(-Constants.TRIGGER_SPEED);
            Robot.shooterIntake.stopPulse();
        } else {
            Robot.shooterIntake.setTrigger(0);
            Robot.shooterIntake.stopPulse();
        }
    }
}
