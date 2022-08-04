package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class LimelightTOCom extends CommandBase{

    private boolean prev_StartButton = false;

    public LimelightTOCom(){
        addRequirements(Robot.limelight);
    }

    @Override
    public void execute(){
        Robot.limelight.updateData();
        Robot.limelight.getRange();
        Robot.limelight.getDistance();
        if(Robot.controller0.getButton(Constants.BUTTON_START) != prev_StartButton){
            prev_StartButton = Robot.controller0.getButton(Constants.BUTTON_START);
            if(Robot.controller0.getButton(Constants.BUTTON_START)){
                Robot.limelight.switchCameraMode();
            }
        }
    }
}