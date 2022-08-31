package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PlayerConfigs;
import frc.robot.Robot;

public class ShooterTOCom extends CommandBase{

    public ShooterTOCom(){
        addRequirements(Robot.shooter);
    }

    @Override
    public void execute(){
        Robot.shooter.setShooterMotor(
            PlayerConfigs.lowPowerShooter ? Constants.SHOOTER_LOW_SPEED : 
                (PlayerConfigs.highPowerShooter ? Constants.SHOOTER_HI_SPEED :
                    (PlayerConfigs.midPowerShooter ? Constants.SHOOTER_MID_SPEED :
                        (!Robot.climbers.getClimbMode() ? Robot.shooter.shooterSpeedAdjust(Robot.limelight.getDistance()) : 0
                    )
                )
            )
        );
        Robot.shooter.updateDashboard();
    }
}