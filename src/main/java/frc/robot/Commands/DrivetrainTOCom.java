package frc.robot.Commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PlayerConfigs;
import frc.robot.Robot;

public class DrivetrainTOCom extends CommandBase{

    public DrivetrainTOCom() {
        addRequirements(Robot.drivetrain);
    }

    @Override
    public void execute(){
        double fLeftMotorSet = 0;
        double bLeftMotorSet = 0;
        double fRightMotorSet = 0;
        double bRightMotorSet = 0;

        // TANK DRIVE
        fLeftMotorSet = 12 * PlayerConfigs.driveSpeed * (PlayerConfigs.accelerator - (PlayerConfigs.steering * PlayerConfigs.turnSpeed));
        fRightMotorSet = 12 * PlayerConfigs.driveSpeed * (PlayerConfigs.accelerator + (PlayerConfigs.steering * PlayerConfigs.turnSpeed));
        bLeftMotorSet = fLeftMotorSet;
        bRightMotorSet = fRightMotorSet;

        //Set motors
        Robot.drivetrain.driveVolts(fLeftMotorSet, bLeftMotorSet, fRightMotorSet, bRightMotorSet);
    }
}