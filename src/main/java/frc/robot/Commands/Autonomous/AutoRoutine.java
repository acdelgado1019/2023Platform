package frc.robot.Commands.Autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.HDD;
import frc.robot.Robot;
import frc.robot.Robot.AutoSection;
import frc.robot.HDD.DesiredMode;

public class AutoRoutine {

    public static double timeCheck = 0.0;

    public static void runAutonomous(){
        switch(Robot.autoSection){
            case STARTUP :
                var pidOutput = Robot.intake.Lift_controller.calculate(
                Robot.intake.getEncoder(), 
                Units.degreesToRadians(Constants.loILPositionDeg));
                Robot.intake.setIntakeLift(pidOutput);
                Robot.shooter.setShooterMotor(Robot.shooter.shooterSpeedAdjust(Robot.limelight.getDistance()));
                if(Timer.getFPGATimestamp() - timeCheck > 1.5){Robot.autoSection = AutoSection.OPENING_ACTION;} 
                break;     
            case OPENING_ACTION :
                if (HDD.desiredMode == DesiredMode.BACK_UP ||
                HDD.desiredMode == DesiredMode.ONE_BALL_RED || HDD.desiredMode == DesiredMode.ONE_BALL_BLUE){
                    AutoMethods.limelightShoot();
                } else if (HDD.desiredMode == DesiredMode.TWO_BALL_RED || HDD.desiredMode == DesiredMode.TWO_BALL_BLUE){
                    Robot.intake.setHorizontalIntake(Constants.HORIZONTAL_INTAKE_SPEED);
                } else if (HDD.desiredMode == DesiredMode.THREE_BALL_RED || HDD.desiredMode == DesiredMode.THREE_BALL_BLUE) {
                    AutoMethods.limelightShoot();
                    Robot.intake.setHorizontalIntake(Constants.HORIZONTAL_INTAKE_SPEED);
                }
                Robot.ledStrip.stripeRB();
                AutoMethods.runRamsete().schedule();
                Robot.autoSection = AutoSection.CLOSING_ACTION;
                timeCheck = Timer.getFPGATimestamp();
                break;
            case CLOSING_ACTION :
            
                if (HDD.desiredMode == DesiredMode.BACK_UP){
                    Robot.autoSection = AutoSection.FINISH;
                } else if (HDD.desiredMode == DesiredMode.ONE_BALL_RED || HDD.desiredMode == DesiredMode.ONE_BALL_BLUE){
                    if(Timer.getFPGATimestamp() - timeCheck > 3.5){
                        Robot.intake.setHorizontalIntake(-Constants.HORIZONTAL_INTAKE_SPEED);
                        Robot.autoSection = AutoSection.FINISH;
                    }
                } else if (HDD.desiredMode == DesiredMode.TWO_BALL_RED || HDD.desiredMode == DesiredMode.TWO_BALL_BLUE){
                    if(Timer.getFPGATimestamp() - timeCheck > 3){
                        Robot.intake.setHorizontalIntake(0);
                        AutoMethods.limelightShoot();
                        Robot.autoSection = AutoSection.FINISH;
                    }
                } else if (HDD.desiredMode == DesiredMode.THREE_BALL_RED || HDD.desiredMode == DesiredMode.THREE_BALL_BLUE) {
                    if(Timer.getFPGATimestamp() - timeCheck > 5){
                        Robot.intake.setHorizontalIntake(0);
                        AutoMethods.limelightShoot();
                        Robot.autoSection = AutoSection.FINISH;
                    }
                }
                break;
            case FINISH :
                Robot.ledStrip.rainbow();
                break;
            case EXIT_AUTO:
            break;
        }
    }
}
