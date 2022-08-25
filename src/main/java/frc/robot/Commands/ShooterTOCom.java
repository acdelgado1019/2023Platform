package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterTOCom extends CommandBase{

    public ShooterTOCom(){
        addRequirements(Robot.shooter);
    }

    @Override
    public void execute(){
        boolean controller1_buttonA = Robot.controller1.getButton(Constants.BUTTON_A);
        boolean controller1_buttonX = Robot.controller1.getButton(Constants.BUTTON_X);
        boolean controller1_buttonY = Robot.controller1.getButton(Constants.BUTTON_Y);

        Robot.shooter.setShooterMotor(
            controller1_buttonA ? Constants.SHOOTER_LOW_SPEED : 
                (controller1_buttonY ? Constants.SHOOTER_HI_SPEED :
                    (controller1_buttonX ? Constants.SHOOTER_MID_SPEED :
                        (!Robot.climbers.getClimbMode() ? Robot.shooter.shooterSpeedAdjust(Robot.limelight.getDistance()) : 0
                    )
                )
            )
        );

        Robot.shooter.updateDashboard();
    }
}