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
        boolean controller1_buttonB = Robot.controller1.getButton(Constants.BUTTON_B);
        boolean controller1_buttonX = Robot.controller1.getButton(Constants.BUTTON_X);
        boolean controller1_buttonY = Robot.controller1.getButton(Constants.BUTTON_Y);

        if(controller1_buttonA == true){
            Robot.shooter.setShooterMotor(Robot.shooter.shooterSpeedAdjust(Robot.limelight.getDistance()));
            Robot.shooter.limelightTrack();
        }else if(controller1_buttonB == true){
            Robot.shooter.setShooterMotor(Constants.SHOOTER_LOW_SPEED);
        }else if(controller1_buttonX == true){
            Robot.shooter.setShooterMotor(Constants.SHOOTER_HI_SPEED);
        }else if(controller1_buttonY == true){
            Robot.shooter.setShooterMotor(Constants.SHOOTER_MID_SPEED);
        }else if(!Robot.climbers.getClimbMode()){
            Robot.shooter.setShooterMotor(Robot.shooter.shooterSpeedAdjust(Robot.limelight.getDistance()));
        }else {Robot.shooter.setShooterMotor(0);}
        Robot.shooter.updateDashboard();
    }
}