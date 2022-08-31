package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PlayerConfigs;
import frc.robot.Robot;

public class IntakeTOCom extends CommandBase{

    public IntakeTOCom(){
        addRequirements(Robot.intake);
    }

    @Override
    public void execute(){
        Robot.intake.setHorizontalIntake(PlayerConfigs.acceptIntake ? Constants.HORIZONTAL_INTAKE_SPEED : (PlayerConfigs.rejectIntake ? -Constants.HORIZONTAL_INTAKE_SPEED : 0));
        
        Robot.intake.setIntakeLift(PlayerConfigs.intakeLiftHi ? Constants.hiILPositionDeg : (PlayerConfigs.intakeLiftMid ? Constants.midILPositionDeg : Constants.loILPositionDeg));

        if (PlayerConfigs.fireTrigger){
            Robot.intake.pulse();
        } else if (PlayerConfigs.rejectTrigger){
            Robot.intake.setTrigger(-Constants.TRIGGER_SPEED);
        } else if (!Robot.climbers.getClimbMode() && Robot.autoSection == Robot.AutoSection.EXIT_AUTO){
            Robot.intake.setTrigger(0);
            Robot.intake.stopPulse();
            Robot.ledStrip.teamColor(Constants.teamColor);
        }
    }
}
