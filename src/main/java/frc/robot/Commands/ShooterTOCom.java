package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PlayerConfigs;
import frc.robot.Robot;

public class ShooterTOCom extends CommandBase{
    private boolean prev_Button = false;

    public ShooterTOCom(){
        addRequirements(Robot.shooter);
    }

    @Override
    public void execute(){
        //Fire Go/No Go Logic
        if (PlayerConfigs.fireTrigger){
            Robot.shooter.setTrigger(Constants.TRIGGER_SPEED, true);
        } else if (PlayerConfigs.rejectTrigger){
            Robot.shooter.setTrigger(-Constants.TRIGGER_SPEED, false);
        } else if (PlayerConfigs.autoTarget && Robot.shooter.getAutoShootEnable()){
            if(Robot.limelight.getRange() && Robot.drivetrain.getStopped()){
                Robot.shooter.setTrigger(Constants.TRIGGER_SPEED, true);
            }
        } else if (!Robot.climbers.getClimbMode() && Robot.autoSection == Robot.AutoSection.EXIT_AUTO){
            Robot.shooter.setTrigger(0, false);
            Robot.ledStrip.teamColor(Constants.teamColor);
        }
        
        //Shooter Speed
        Robot.shooter.setShooterMotor(
            PlayerConfigs.lowPowerShooter ? Constants.SHOOTER_LOW_SPEED : 
                (PlayerConfigs.highPowerShooter ? Constants.SHOOTER_HI_SPEED :
                    (PlayerConfigs.midPowerShooter ? Constants.SHOOTER_MID_SPEED :
                        (!Robot.climbers.getClimbMode() ? Robot.shooter.shooterSpeedAdjust(Robot.limelight.getDistance()) : 0
                    )
                )
            )
        );

        if(PlayerConfigs.changeAutoShootState != prev_Button){
            prev_Button = PlayerConfigs.changeAutoShootState;
            if(PlayerConfigs.changeAutoShootState){
                Robot.shooter.changeAutoShootState();
            }
        }
        Robot.shooter.updateDashboard();
    }
}