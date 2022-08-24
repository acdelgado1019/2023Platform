package frc.robot.Commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeTOCom extends CommandBase{

    public IntakeTOCom(){
        addRequirements(Robot.intake);
    }

    @Override
    public void execute(){
        //lift
        boolean controller0_leftBumper = Robot.controller0.getButton(Constants.LEFT_BUMPER);
        boolean controller0_leftTrigger = Robot.controller0.getButton(Constants.LEFT_TRIGGER);

        //horizontal
        boolean controller0_rightBumper = Robot.controller0.getButton(Constants.RIGHT_BUMPER);
        boolean controller0_rightTrigger = Robot.controller0.getButton(Constants.RIGHT_TRIGGER);
        
        //vertical
        boolean controller1_leftBumper = Robot.controller1.getButton(Constants.LEFT_BUMPER);
        boolean controller1_leftTrigger = Robot.controller1.getButton(Constants.LEFT_TRIGGER);

        Robot.intake.setHorizontalIntake(controller1_leftTrigger ? Constants.HORIZONTAL_INTAKE_SPEED : (controller1_leftBumper ? -Constants.HORIZONTAL_INTAKE_SPEED : 0));
        
        if (controller0_rightBumper){
            var pidOutput =
                Robot.intake.Lift_controller.calculate(Robot.intake.getEncoder(), Units.degreesToRadians(Constants.midILPositionDeg));
            Robot.intake.setIntakeLift(pidOutput);
        } else if(controller0_rightTrigger){
            var pidOutput =
                Robot.intake.Lift_controller.calculate(Robot.intake.getEncoder(), Units.degreesToRadians(Constants.hiILPositionDeg));
            Robot.intake.setIntakeLift(pidOutput);
        } else{
            var pidOutput =
                Robot.intake.Lift_controller.calculate(Robot.intake.getEncoder(), Units.degreesToRadians(Constants.loILPositionDeg));
            Robot.intake.setIntakeLift(pidOutput);
        }

        if (controller0_leftBumper){
            Robot.intake.pulse();
        } else if (controller0_leftTrigger){
            Robot.intake.setTrigger(-Constants.TRIGGER_SPEED);
        } else if (!Robot.climbers.getClimbMode() && Robot.autoSection == Robot.AutoSection.EXIT_AUTO){
            Robot.intake.setTrigger(0);
            Robot.intake.stopPulse();
            Robot.ledStrip.teamColor(Constants.teamColor);
        }
        
    }
}
