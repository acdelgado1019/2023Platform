package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DrivetrainTOCom extends CommandBase{

    public DrivetrainTOCom() {
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void execute(){
        //Get joystick values
        double controller0_rightStickX = Robot.controller0.getJoystickAxis(Constants.RIGHT_STICK_X);
        double controller0_leftStickY = Robot.controller0.getJoystickAxis(Constants.LEFT_STICK_Y);

        double leftMotorSet = 0;
        double rightMotorSet = 0;

        // Calculate values to set motors
        leftMotorSet = ((controller0_leftStickY - (Constants.LEFT_RIGHT_TRIM + (controller0_rightStickX * Constants.MAX_TURN_SPEED))) * Constants.MAX_DRIVE_SPEED);
        rightMotorSet = ((controller0_leftStickY + (Constants.LEFT_RIGHT_TRIM + (controller0_rightStickX * Constants.MAX_TURN_SPEED))) * Constants.MAX_DRIVE_SPEED);

        //Set motors
        Robot.drivetrain.setLeftDrivetrain(leftMotorSet);
        Robot.drivetrain.setRightDrivetrain(rightMotorSet);

        if (Robot.controller0.getButton(Constants.BUTTON_A)){AutoMethods.rotate(2);}
    }
}