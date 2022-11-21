package frc.robot.Commands;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.PlayerConfigs;
import frc.robot.Robot;

public class DrivetrainTOCom extends CommandBase{

    private boolean driveMode = true;
    double prevFLeft = 0;
    double prevBLeft = 0;
    double prevFRight = 0;
    double prevBRight = 0;
    double turnSpeed;
    private MecanumDriveMotorVoltages driveVolts = new MecanumDriveMotorVoltages(0, 0, 0, 0);

    public DrivetrainTOCom() {
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void execute(){

        turnSpeed = PlayerConfigs.turnMovement * PlayerConfigs.turnSpeed;

        if(driveMode){
            //Mecanum Drive
            driveVolts.frontLeftVoltage = DriveConstants.RAMP_RATE * (12 * PlayerConfigs.driveSpeed * (PlayerConfigs.yMovement + PlayerConfigs.xMovement + turnSpeed)) + (1 - DriveConstants.RAMP_RATE) * prevFLeft;
            driveVolts.frontRightVoltage = DriveConstants.RAMP_RATE * (12 * PlayerConfigs.driveSpeed * (PlayerConfigs.yMovement - PlayerConfigs.xMovement - turnSpeed)) + (1 - DriveConstants.RAMP_RATE) * prevFRight;
            driveVolts.rearLeftVoltage = DriveConstants.RAMP_RATE * (12 * PlayerConfigs.driveSpeed * (PlayerConfigs.yMovement - PlayerConfigs.xMovement + turnSpeed)) + (1 - DriveConstants.RAMP_RATE) * prevBLeft;
            driveVolts.rearRightVoltage = DriveConstants.RAMP_RATE * (12 * PlayerConfigs.driveSpeed * (PlayerConfigs.yMovement + PlayerConfigs.xMovement - turnSpeed)) + (1 - DriveConstants.RAMP_RATE) * prevBRight;
        } else {
            //Tank Drive
            driveVolts.frontLeftVoltage = DriveConstants.RAMP_RATE * (12 * PlayerConfigs.driveSpeed * (PlayerConfigs.yMovement - turnSpeed)) + (1 - DriveConstants.RAMP_RATE) * prevFLeft;
            driveVolts.frontRightVoltage = DriveConstants.RAMP_RATE * (12 * PlayerConfigs.driveSpeed * (PlayerConfigs.yMovement + turnSpeed)) + (1 - DriveConstants.RAMP_RATE) * prevFRight;
            driveVolts.rearLeftVoltage = driveVolts.frontLeftVoltage;
            driveVolts.rearRightVoltage = driveVolts.frontRightVoltage;
        }

        prevFLeft = driveVolts.frontLeftVoltage;
        prevBLeft = driveVolts.rearLeftVoltage;
        prevFRight = driveVolts.frontRightVoltage;
        prevBRight = driveVolts.rearRightVoltage;

        if (PlayerConfigs.modeSwitchMec){
            driveMode = true;
        } else if (PlayerConfigs.modeSwitchTank){
            driveMode = false;
        }

        //Set motors
        Robot.drivetrain.setDriveMotorControllersVolts(driveVolts);
        
    }
}