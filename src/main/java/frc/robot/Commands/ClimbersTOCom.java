package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PlayerConfigs;
import frc.robot.Robot;
import frc.robot.Subsystems.Climbers.AutoClimbStep;

public class ClimbersTOCom extends CommandBase{    

    public ClimbersTOCom(){
        addRequirements(Robot.climbers);
    }

    @Override
    public void execute(){   
        int val = (int)Math.max(Math.min(-Robot.climbers.getLeftClimbEncoder() * 8,255.0),0.0);


        if(PlayerConfigs.climbModeReset){
            Robot.climbers.resetClimbMode();
            Robot.climbers.autoClimbStep = AutoClimbStep.MANUAL_MODE;
        }

        if(PlayerConfigs.autoClimbTriggerA && PlayerConfigs.autoClimbTriggerB){
            Robot.climbers.autoClimbStep = AutoClimbStep.MID_BAR_RETRACT;
        }

        switch (Robot.climbers.autoClimbStep){
            case MANUAL_MODE :
                if(PlayerConfigs.climberExtend){
                    Robot.climbers.setClimbMode();
                    Robot.climbers.setLeftClimber(Constants.CLIMBER_MOVEMENT_SPEED);
                    Robot.climbers.setRightClimber(Constants.CLIMBER_MOVEMENT_SPEED);
                    Robot.ledStrip.rainbow();
                } else if (PlayerConfigs.climberRetract) {
                    Robot.climbers.setClimbMode();
                    Robot.climbers.setLeftClimber(-Constants.CLIMBER_MOVEMENT_SPEED);
                    Robot.climbers.setRightClimber(-Constants.CLIMBER_MOVEMENT_SPEED);
                    Robot.ledStrip.rainbow();
                } else if (Robot.climbers.getClimbMode()){
                    Robot.climbers.setLeftClimber(PlayerConfigs.climberLeftExtension*Constants.CLIMBER_MOVEMENT_SPEED);
                    Robot.climbers.setRightClimber(PlayerConfigs.climberRightExtension*Constants.CLIMBER_MOVEMENT_SPEED);
                    Robot.ledStrip.rainbow();
                } else {
                    Robot.climbers.setLeftClimber(0);
                    Robot.climbers.setRightClimber(0);
                }
            

                if (PlayerConfigs.climberRotate){
                    Robot.climbers.setClimberRotation(Constants.RotatorFullPositionDeg);
                } else if(PlayerConfigs.climberExtend){
                    Robot.climbers.setClimberRotation(Constants.RotatorUnhookPositionDeg);
                } else {
                    Robot.climbers.setClimberRotation(Constants.RotatorVerticalPositionDeg);
                }
                break;
            case MID_BAR_RETRACT :
                Robot.ledStrip.varSat(255,val);
                Robot.climbers.setClimberRotation(Constants.RotatorVerticalPositionDeg);
                Robot.climbers.setLeftClimber(-Constants.CLIMBER_MOVEMENT_SPEED);
                Robot.climbers.setRightClimber(-Constants.CLIMBER_MOVEMENT_SPEED);
                if (Robot.climbers.getLeftClimbEncoder() > -3 && Robot.climbers.getRightClimbEncoder() > -3){
                    Robot.climbers.autoClimbStep = AutoClimbStep.MID_BAR_RELEASE;
                }
                break;
            case MID_BAR_RELEASE :
                Robot.ledStrip.varSat(255,val);
                Robot.climbers.setClimberRotation(Constants.RotatorUnhookPositionDeg);
                Robot.climbers.setLeftClimber(Constants.CLIMBER_MOVEMENT_SPEED);
                Robot.climbers.setRightClimber(Constants.CLIMBER_MOVEMENT_SPEED);
                if (Robot.climbers.getLeftClimbEncoder() < -30 && Robot.climbers.getRightClimbEncoder() < -30){
                    Robot.climbers.autoClimbStep = AutoClimbStep.HIGH_BAR_EXTEND;
                }
                break;
            case HIGH_BAR_EXTEND :
                Robot.ledStrip.varSat(255,val);
                Robot.climbers.setClimberRotation(Constants.RotatorFullPositionDeg);
                Robot.climbers.setLeftClimber(Constants.CLIMBER_MOVEMENT_SPEED);
                Robot.climbers.setRightClimber(Constants.CLIMBER_MOVEMENT_SPEED);
                if (Robot.climbers.getLeftClimbEncoder() < -270 && Robot.climbers.getRightClimbEncoder() < -270){
                    Robot.climbers.autoClimbStep = AutoClimbStep.HIGH_BAR_RETRACT;
                }
                break;
            case HIGH_BAR_RETRACT :
                Robot.ledStrip.varSat(255,val);
                Robot.climbers.setClimberRotation(Constants.RotatorVerticalPositionDeg);
                Robot.climbers.setLeftClimber(-Constants.CLIMBER_MOVEMENT_SPEED);
                Robot.climbers.setRightClimber(-Constants.CLIMBER_MOVEMENT_SPEED);
                if (Robot.climbers.getLeftClimbEncoder() > -3 && Robot.climbers.getRightClimbEncoder() > -3){
                    Robot.climbers.autoClimbStep = AutoClimbStep.HIGH_BAR_RELEASE;
                }
                break;
            case HIGH_BAR_RELEASE :
                Robot.ledStrip.varSat(255-val,val);
                Robot.climbers.setClimberRotation(Constants.RotatorUnhookPositionDeg);
                Robot.climbers.setLeftClimber(Constants.CLIMBER_MOVEMENT_SPEED);
                Robot.climbers.setRightClimber(Constants.CLIMBER_MOVEMENT_SPEED);
                if (Robot.climbers.getLeftClimbEncoder() < -30 && Robot.climbers.getRightClimbEncoder() < -30){
                    Robot.climbers.autoClimbStep = AutoClimbStep.TRAVERSAL_BAR_EXTEND;
                }
                break;
            case TRAVERSAL_BAR_EXTEND :
                Robot.ledStrip.varSat(255-val,val);
                Robot.climbers.setClimberRotation(Constants.RotatorFullPositionDeg);
                Robot.climbers.setLeftClimber(Constants.CLIMBER_MOVEMENT_SPEED);
                Robot.climbers.setRightClimber(Constants.CLIMBER_MOVEMENT_SPEED);
                if (Robot.climbers.getLeftClimbEncoder() < -270 && Robot.climbers.getRightClimbEncoder() < -270){
                    Robot.climbers.autoClimbStep = AutoClimbStep.TRAVERSAL_BAR_RETRACT;
                }
                break;
            case TRAVERSAL_BAR_RETRACT :
                Robot.ledStrip.varRainbow(255-val/2);
                Robot.climbers.setClimberRotation(Constants.RotatorVerticalPositionDeg);
                Robot.climbers.setLeftClimber(-Constants.CLIMBER_MOVEMENT_SPEED);
                Robot.climbers.setRightClimber(-Constants.CLIMBER_MOVEMENT_SPEED);
                if (Robot.climbers.getLeftClimbEncoder() > -200 && Robot.climbers.getRightClimbEncoder() > -200){
                    Robot.climbers.autoClimbStep = AutoClimbStep.CLIMB_COMPLETE;
                }
                break;
            case CLIMB_COMPLETE : 
                Robot.ledStrip.rainbow();
                Robot.climbers.setLeftClimber(0);
                Robot.climbers.setRightClimber(0);
                Robot.climbers.setClimberRotation(Constants.RotatorVerticalPositionDeg);
                break;
        }
        Robot.climbers.updateDashboard();
    }
}